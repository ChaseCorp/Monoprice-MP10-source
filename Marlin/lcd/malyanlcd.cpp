/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * malyanlcd.cpp
 *
 * LCD implementation for Malyan's LCD, a separate ESP8266 MCU running
 * on Serial1 for the M200 board. This module outputs a pseudo-gcode
 * wrapped in curly braces which the LCD implementation translates into
 * actual G-code commands.
 *
 * Added to Marlin for Mini/Malyan M200
 * Unknown commands as of Jan 2018: {H:}
 * Not currently implemented:
 * {E:} when sent by LCD. Meaning unknown.
 *
 * Notes for connecting to boards that are not Malyan:
 * The LCD is 3.3v, so if powering from a RAMPS 1.4 board or
 * other 5v/12v board, use a buck converter to power the LCD and
 * the 3.3v side of a logic level shifter. Aux1 on the RAMPS board
 * has Serial1 and 12v, making it perfect for this.
 * Copyright (c) 2017 Jason Nelson (xC0000005)
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(MALYAN_LCD)

#include "../sd/cardreader.h"
#include "../sd/SdFatConfig.h"
#include "../module/temperature.h"
#include "../module/planner.h"
#include "../module/stepper.h"
#include "../module/motion.h"
#include "../module/probe.h"
#include "../libs/duration_t.h"
#include "../module/printcounter.h"
#include "../gcode/gcode.h"
#include "../gcode/queue.h"
#include "../module/configuration_store.h"

#include "../Marlin.h"
#include "ubl.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../feature/power_loss_recovery.h"
#endif

int32_t sd_err_cnt;
uint8_t save_later=0;
uint32_t save_later_time;
static uint8_t last_endstop_stat=0;

// On the Malyan M200, this will be Serial1. On a RAMPS board,
// it might not be.
#define LCD_SERIAL Serial1

#include "HAL_Stm32f4.h"

#define WIFI_UART       USART1
void wifi_put(char c)
{
#if SIMULATE_DEBUG
    return;
#endif
    while ((USART1->SR & USART_SR_TXE) == 0) {}
    USART1->DR = c+0x80;
}

void wifi_write(char s[], uint8_t l)
{
    uint8_t i;
    for (i=0;i<l;i++) wifi_put(s[i]);
}

// This is based on longest sys command + a filename, plus some buffer
// in case we encounter some data we don't recognize
// There is no evidence a line will ever be this long, but better safe than sory
#define MAX_CURLY_COMMAND (32 + LONG_FILENAME_LENGTH) * 2

// Track incoming command bytes from the LCD
int inbound_count;

// Everything written needs the high bit set.
void write_to_lcd_P(const char * const message) {
  wifi_write((char *)message, strlen(message));
  return;
}

void write_to_lcd(const char * const message) {
  wifi_write((char *)message, strlen(message));
  return;
  char encoded_message[MAX_CURLY_COMMAND];
  const uint8_t message_length = min(strlen(message), sizeof(encoded_message));

  for (uint8_t i = 0; i < message_length; i++)
    encoded_message[i] = message[i];

  wifi_write(encoded_message, message_length);
}

void card_read_err_message()
{
    char message_buffer[MAX_CURLY_COMMAND];
    sprintf_P(message_buffer,"{ERR:CARD=%d}",sd_err_cnt);
    write_to_lcd(message_buffer);
}

uint8_t lcd_cancel_print_flag=0;
extern void reboot();
extern "C" void lcd_cancel_print()
{
    write_to_lcd_P(PSTR("{SYS:CANCELING}"));
    card.stopSDPrint();
    clear_command_queue();
    quickstop_stepper();
    print_job_timer.stop();
    thermalManager.disable_all_heaters();
    #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
    #endif
    wait_for_heatup = false;
    write_to_lcd_P(PSTR("{SYS:STARTED}"));
    
    lcd_cancel_print_flag=0;
    
    #if ENABLED(POWER_LOSS_RECOVERY)
        job_recovery_info.valid_head = job_recovery_info.valid_foot = 0;
        job_recovery_commands_count = 0;
    #endif
    reboot();
}

#if ENABLED(POWER_LOSS_RECOVERY)

    void lcd_sdcard_recover_job() {
      char cmd[20];

      #if HAS_HEATED_BED
        // Restore the bed temperature
        sprintf_P(cmd, PSTR("M190 S%i"), job_recovery_info.target_temperature_bed);
        enqueue_and_echo_command(cmd);
      #endif

      // Restore all hotend temperatures
      HOTEND_LOOP() {
        sprintf_P(cmd, PSTR("M109 S%i"), job_recovery_info.target_temperature[e]);
        enqueue_and_echo_command(cmd);
      }

      // Restore print cooling fan speeds
      for (uint8_t i = 0; i < FAN_COUNT; i++) {
        sprintf_P(cmd, PSTR("M106 P%i S%i"), i, job_recovery_info.fanSpeeds[i]);
        enqueue_and_echo_command(cmd);
      }
      
      // Start draining the job recovery command queue
      job_recovery_phase = JOB_RECOVERY_YES;

      // Start getting commands from SD
      card.startFileprint();
    }

#endif // POWER_LOSS_RECOVERY

void lcd_write_version()
{
    write_to_lcd_P(PSTR("{VER:"));
    write_to_lcd_P(PSTR(SHORT_BUILD_VERSION));
    write_to_lcd_P(PSTR("}\r\n"));
}

void lcd_homing_done()
{
    //write_to_lcd_P("{VER:68}\r\n");
    
}

/**
 * Process an LCD 'C' command.
 * These are currently all temperature commands
 * {C:T0190}
 * Set temp for hotend to 190
 * {C:P050}
 * Set temp for bed to 50
 *
 * the command portion begins after the :
 */
void process_lcd_c_command(const char* command) {
  switch (command[0]) {
    case 'T': {
      // M104 S<temperature>
        uint16_t t;
        t=atoi(command + 1);
        if (t>300) t=0;
        thermalManager.setTargetHotend(t, 0);
    } break;

    case 'P': {
      // M140 S<temperature>
        uint16_t t;
        t=atoi(command + 1);
        if (t>150) t=0;
        thermalManager.setTargetBed(t);
    } break;

    case 'S': {
        uint16_t t;
        t=atoi(command + 1);
        if (t>50) t=50;
        else if (t<1) t=1;
        feedrate_percentage=t*10;
    } break;

    default:
      SERIAL_ECHOLNPAIR("UNKNOWN C COMMAND", command);
      return;
  }
}

/**
 * Process an LCD 'B' command.
 * {B:0} results in: {T0:008/195}{T1:000/000}{TP:000/000}{TQ:000C}{TT:000000}
 * T0/T1 are hot end temperatures, TP is bed, TQ is percent, and TT is probably
 * time remaining (HH:MM:SS). The UI can't handle displaying a second hotend,
 * but the stock firmware always sends it, and it's always zero.
 */
void process_lcd_eb_command(const char* command) {
  char elapsed_buffer[10];
  duration_t elapsed;
  bool has_days;
  uint8_t len;
  switch (command[0]) {
    case '0': {
      elapsed = print_job_timer.duration();
      sprintf_P(elapsed_buffer, PSTR("%02u%02u%02u"), uint16_t(elapsed.hour()), uint16_t(elapsed.minute()) % 60UL, elapsed.second());

      char message_buffer[MAX_CURLY_COMMAND];
      sprintf_P(message_buffer,
              PSTR("{T0:%03.0f/%03i}{T1:000/000}{TP:%03.0f/%03i}{TQ:%03i}{TT:%s}"),
              thermalManager.degHotend(0),
              thermalManager.degTargetHotend(0),
              thermalManager.degBed(),
              thermalManager.degTargetBed(),
              card.percentDone(),
              elapsed_buffer);
      write_to_lcd(message_buffer);
    } break;

    default:
      SERIAL_ECHOLNPAIR("UNKNOWN E/B COMMAND", command);
      return;
  }
}

/**
 * Process an LCD 'J' command.
 * These are currently all movement commands.
 * The command portion begins after the :
 * Move X Axis
 *
 * {J:E}{J:X-200}{J:E}
 * {J:E}{J:X+200}{J:E}
 * X, Y, Z, A (extruder)
 */
void process_lcd_j_command(const char* command) {
  extern bool relative_mode;
  extern bool enqueuecommand(const char* cmd, bool say_ok=false);
  char cmd[16];
  extern bool relative_mode;
  switch (command[0])
  {
      case 'E':
          //extern unsigned int cleaning_buffer_counter;
          //enqueuecommands_P(PSTR("M18"));
          clear_command_queue();
          quickstop_stepper();
          //while (cleaning_buffer_counter>0) IWDG_ReloadCounter();
          relative_mode=false;
          disable_all_steppers();
          
          for (int i = 0; i < XYZE; i++) destination[i] = current_position[i];
          
          break;

      case 'X':
      case 'Y':
      case 'Z':
          extern float current_position[NUM_AXIS],destination[NUM_AXIS];
          extern float feedrate_mm_s;
          
          for (int i = 0; i < XYZE; i++)  destination[i] = current_position[i];

          if (command[0]=='X')
          {
              if (command[1]=='+') destination[0] = current_position[0] + 300;
              else
              {
                  if (READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING) return;
                  destination[0] = current_position[0] -300;
              }
          }
          else if (command[0]=='Y')
          {
              if (command[1]=='+') destination[1] = current_position[1] + 300;
              else
              {
                  if (READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING) return;
                  destination[1] = current_position[1] -300;
              }
          }
          else if (command[0]=='Z')
          {
              if (command[1]=='+') destination[2] = current_position[2] + 120;
              else
              {
                  if (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING) return;
                  destination[2] = current_position[2] -120;
              }
          }
          
          enable_all_steppers();
          relative_mode=true;
          feedrate_mm_s = MMM_TO_MMS(1000);
          buffer_line_to_destination(MMS_SCALED(feedrate_mm_s));
        break;
      case 'A':
      case 'B':
          relative_mode=true;

          if (command[0]=='A') enqueuecommand("T0");
          else enqueuecommand("T1");

          strcpy(cmd,"G1 F1000 ");
          cmd[9]=command[0];   //X
          cmd[10]=command[1];   //+
          if (cmd[9]=='A')
          {
              cmd[9]='E';
          }
          else if (cmd[9]=='B')
          {
              cmd[9]='E';
          }
          cmd[11]=command[2];   //1
          cmd[12]=command[3];  //0
          cmd[13]=command[4];  //0
          cmd[14]=0;
          enqueuecommand(cmd);

          break;
  }
  return;
  
  char axis = command[0];

  switch (axis) {
    case 'E':
      
      break;
    case 'A':
      axis = 'E';
      // fallthru
    case 'Y':
    case 'Z':
    case 'X': {
      // G0 <AXIS><distance>
      // The M200 class UI seems to send movement in .1mm values.
      char cmd[20];
      relative_mode=true;
      sprintf_P(cmd, PSTR("G1 F1000 %c%03.1f"), axis, atof(command + 1) / 10.0);
      enqueuecommand(cmd);
    } break;
    default:
      SERIAL_ECHOLNPAIR("UNKNOWN J COMMAND", command);
      return;
  }
}

/**
 * Process an LCD 'P' command, related to homing and printing.
 * Cancel:
 * {P:X}
 *
 * Home all axes:
 * {P:H}
 *
 * Print a file:
 * {P:000}
 * The File number is specified as a three digit value.
 * Printer responds with:
 * {PRINTFILE:Mini_SNES_Bottom.gcode}
 * {SYS:BUILD}echo:Now fresh file: Mini_SNES_Bottom.gcode
 * File opened: Mini_SNES_Bottom.gcode Size: 5805813
 * File selected
 * {SYS:BUILD}
 * T:-2526.8 E:0
 * T:-2533.0 E:0
 * T:-2537.4 E:0
 * Note only the curly brace stuff matters.
 */
void process_lcd_p_command(const char* command) {

  switch (command[0]) {
    case 'X':
      // cancel print
      lcd_cancel_print_flag=1;
      break;
    case 'P':
      write_to_lcd_P(PSTR("{SYS:PAUSE}"));
      //enqueue_and_echo_command("M25");
      //extern void M25();
      card.pauseSDPrint();
      extern uint8_t did_pause_print;
      ++did_pause_print;
      print_job_timer.pause();

      #if ENABLED(PARK_HEAD_ON_PAUSE)
        enqueue_and_echo_commands_P(PSTR("M125 Z10")); // Must be enqueued with pauseSDPrint set to be last in the buffer
      #endif
      write_to_lcd_P(PSTR("{SYS:PAUSED}"));
      break;
    case 'R':
      write_to_lcd_P(PSTR("{SYS:RESUME}"));
      enqueue_and_echo_command("M24");
      write_to_lcd_P(PSTR("{SYS:RESUMED}"));
      break;
      
    case 'H':
      // Home all axis
      /*enqueue_and_echo_command("G92 X0 Y0 Z0\n", false);
      enqueue_and_echo_command("G28 R0 X Y\n", false);*/
      
      enqueue_and_echo_command_now("G28");
      enqueue_and_echo_command_now("G18");
      enqueue_and_echo_command_now("M400");
      break;
      
    case 'I':
      enqueue_and_echo_command_now("G29 P0");
      enqueue_and_echo_command_now("G29 P1");
      enqueue_and_echo_command_now("G29 A");
      enqueue_and_echo_command_now("M500");
      enqueue_and_echo_command_now("M400");
      break;
      
    default: {
      // Print file 000 - a three digit number indicating which
      // file to print in the SD card. If it's a directory,
      // then switch to the directory.
      delay(500);

      // Find the name of the file to print.
      // It's needed to echo the PRINTFILE option.
      // The {S:L} command should've ensured the SD card was mounted.
      uint16_t index = atoi(command);
      if (card.workDirDepth==0) card.getfilename(atoi(command));
      else
      {
          if (index==0)
          {
              card.updir();
              write_to_lcd_P(PSTR("{SYS:DIR}"));
              return;
          }
          else card.getfilename(atoi(command)-1);
      }

      // There may be a difference in how V1 and V2 LCDs handle subdirectory
      // prints. Investigate more. This matches the V1 motion controller actions
      // but the V2 LCD switches to "print" mode on {SYS:DIR} response.
      if (card.filenameIsDir) {
        card.chdir(card.filename);
        write_to_lcd_P(PSTR("{SYS:DIR}"));
      }
      else {
        char message_buffer[MAX_CURLY_COMMAND];
        #define LONGEST_FILENAME (card.longFilename[0] ? card.longFilename : card.filename)
        sprintf_P(message_buffer, PSTR("{PRINTFILE:%s}"), LONGEST_FILENAME);
        if (strstr(card.filename,"RECOVERY.BIN"))
        {
          /*enqueue_and_echo_command("G28 X Y", false);
          enqueue_and_echo_command("G91");*/
          
          do_print_job_recovery();
        }
        else
        {
        write_to_lcd(message_buffer);
        write_to_lcd_P(PSTR("{SYS:BUILD}"));
        write_to_lcd(message_buffer);
        write_to_lcd_P(PSTR("{SYS:BUILD}"));
        card.openFile(card.filename, true);
        card.startFileprint();
        print_job_timer.start();
        }
      }
    } break; // default
  } // switch
}

/**
 * Handle an lcd 'S' command
 * {S:I} - Temperature request
 * {T0:999/000}{T1:000/000}{TP:004/000}
 *
 * {S:L} - File Listing request
 * Printer Response:
 * {FILE:buttons.gcode}
 * {FILE:update.bin}
 * {FILE:nupdate.bin}
 * {FILE:fcupdate.flg}
 * {SYS:OK}
 */
void process_lcd_s_command(const char* command) {
  switch (command[0]) {
    case 'I': {
      // temperature information
      char message_buffer[MAX_CURLY_COMMAND];
      sprintf_P(message_buffer, PSTR("{T0:%03.0f/%03i}{T1:000/000}{TP:%03.0f/%03i}"),
        thermalManager.degHotend(0), thermalManager.degTargetHotend(0),
        thermalManager.degBed(), thermalManager.degTargetBed()
      );
      write_to_lcd(message_buffer);
    } break;

    case 'H':
      // Home all axis
      enqueue_and_echo_command("G28", false);
      break;

    case 'L': {
      /*if (!card.cardOK) */
        card.initsd();

      // A more efficient way to do this would be to
      // implement a callback in the ls_SerialPrint code, but
      // that requires changes to the core cardreader class that
      // would not benefit the majority of users. Since one can't
      // select a file for printing during a print, there's
      // little reason not to do it this way.
      char message_buffer[MAX_CURLY_COMMAND];
      uint16_t file_count = card.get_num_Files();
      delay(100);
      if (card.workDirDepth!=0) write_to_lcd("{DIR:..}");
      for (uint16_t i = 0; i < file_count; i++) {
        card.getfilename(i);
        #define LONGEST_FILENAME (card.longFilename[0] ? card.longFilename : card.filename)
        sprintf_P(message_buffer, card.filenameIsDir ? PSTR("{DIR:%s}") : PSTR("{FILE:%s}"), LONGEST_FILENAME);
        write_to_lcd(message_buffer);
      }

      write_to_lcd_P(PSTR("{SYS:OK}"));
    } break;

    default:
      SERIAL_ECHOLNPAIR("UNKNOWN S COMMAND", command);
      return;
  }
}

void process_lcd_m_command(const char* command) {
    uint16_t t;
    float f;
    t=atoi((const char*)command);
    if (t>=999)
    {
        f=(-zprobe_zoffset*100);
        t=f;
        char message_buffer[MAX_CURLY_COMMAND];
        sprintf_P(message_buffer, "{M:%03i}",t);
        write_to_lcd(message_buffer);
    }
    else
    {
        f=t;
        f=-f*0.01;
        
        if (Z_PROBE_OFFSET_RANGE_MIN <= f && f <= Z_PROBE_OFFSET_RANGE_MAX) {
          float last_zprobe_zoffset = zprobe_zoffset;
          zprobe_zoffset = f;
          
          float diff = zprobe_zoffset - last_zprobe_zoffset;
          for (uint8_t x=0;x<GRID_MAX_POINTS_X;x++)
            for (uint8_t y=0;y<GRID_MAX_POINTS_Y;y++)
            {
              ubl.z_values[x][y] += diff;
            }
          
        }
        
        save_later=1;
        save_later_time=millis()+10000;
    }
}

/**
 * Receive a curly brace command and translate to G-code.
 * Currently {E:0} is not handled. Its function is unknown,
 * but it occurs during the temp window after a sys build.
 */
void process_lcd_command(const char* command) {
  const char *current = command;

  current++; // skip the leading {. The trailing one is already gone.
  byte command_code = *current++;
  if (*current != ':') {
    SERIAL_ECHOPAIR("UNKNOWN COMMAND FORMAT", command);
    SERIAL_ECHOLN("}");
    return;
  }

  current++; // skip the :

  switch (command_code) {
    case 'M':
      process_lcd_m_command(current);
      break;
    case 'S':
      process_lcd_s_command(current);
      break;
    case 'J':
      process_lcd_j_command(current);
      break;
    case 'P':
      process_lcd_p_command(current);
      break;
    case 'C':
      process_lcd_c_command(current);
      break;
    case 'B':
    case 'E':
      process_lcd_eb_command(current);
      break;
    case 'V':
      lcd_write_version();
      break;
    default:
      SERIAL_ECHOLNPAIR("UNKNOWN COMMAND", command);
      return;
  }
}

/**
 * UC means connected.
 * UD means disconnected
 * The stock firmware considers USB initialied as "connected."
 */
extern uint8_t com_opened;
void update_usb_status(const bool forceUpdate) {
  static bool last_usb_connected_status = false;
  // This is mildly different than stock, which
  // appears to use the usb discovery status.
  // This is more logical.
  if (last_usb_connected_status != com_opened || forceUpdate) {
    last_usb_connected_status =  com_opened;
    write_to_lcd_P(last_usb_connected_status ? PSTR("{R:UC}\r\n") : PSTR("{R:UD}\r\n"));
  }
}

/**
 * - from printer on startup:
 * {SYS:STARTED}{VER:29}{SYS:STARTED}{R:UD}
 * The optimize attribute fixes a register Compile
 * error for amtel.
 */
#define WIFI_RX_BUFFER_SIZE 64
//extern struct wifi_ring_buffer wifi_rx_buffer;
extern "C" struct wifi_ring_buffer {
  unsigned char buffer[WIFI_RX_BUFFER_SIZE];
  int head;
  int tail;
} wifi_rx_buffer;

static uint32_t last_endstop_update;
void lcd_update() _O2 {
  static char inbound_buffer[MAX_CURLY_COMMAND];

  // First report USB status.
  update_usb_status(false);
  
    uint8_t endstop_stat=0;
    if (READ(X_STOP_PIN) == 0) endstop_stat|=1;
    //if (READ(PA15) == 0) endstop_stat|=1;
    if (READ(Y_STOP_PIN) == 0) endstop_stat|=2;
    if (READ(Z_STOP_PIN) == 0) endstop_stat|=4;
    
    if (save_later==1)
    {
      if (millis()>save_later_time)
      {
        save_later=0;
        enqueue_and_echo_command("M500", false);
      }
    }
    if ((endstop_stat != last_endstop_stat) && millis()>last_endstop_update)
    {
        write_to_lcd((char *)"{TS:");
        if (endstop_stat&1) wifi_put('X');
        else wifi_put('x');
        if (endstop_stat&2) wifi_put('Y');
        else wifi_put('y');
        if (Z_MIN_ENDSTOP_INVERTING)
        {
            if (endstop_stat&4) wifi_put('Z');
            else wifi_put('z');
        }
        else
        {
            if (endstop_stat&4) wifi_put('z');
            else wifi_put('Z');
        }
        last_endstop_stat=endstop_stat;
        wifi_put('}');
        last_endstop_update=millis()+500;
    }
    
  #if ENABLED(POWER_LOSS_RECOVERY)
    if (job_recovery_commands_count && job_recovery_phase == JOB_RECOVERY_IDLE) {
      //lcd_goto_screen(lcd_job_recovery_menu);
      lcd_sdcard_recover_job();
      job_recovery_phase = JOB_RECOVERY_MAYBE; // Waiting for a response
    }
  #endif


  // now drain commands...
  while ((unsigned int)(WIFI_RX_BUFFER_SIZE + wifi_rx_buffer.head - wifi_rx_buffer.tail) % WIFI_RX_BUFFER_SIZE)
  {
    const byte b = wifi_rx_buffer.buffer[wifi_rx_buffer.tail];
    wifi_rx_buffer.tail = (unsigned int)(wifi_rx_buffer.tail + 1) % WIFI_RX_BUFFER_SIZE;

    inbound_buffer[inbound_count++] = b;
    if (b == '}' || inbound_count == sizeof(inbound_buffer) - 1) {
      inbound_buffer[inbound_count - 1] = '\0';
      process_lcd_command(inbound_buffer);
      inbound_count = 0;
      inbound_buffer[0] = 0;
    }
  }
  // If there's a print in progress, we need to emit the status as
  // {TQ:<PERCENT>}
  if (card.sdprinting) {
    // We also need to send: T:-2538.0 E:0
    // I have no idea what this means.
    char message_buffer[10];
    sprintf_P(message_buffer, PSTR("{TQ:%03i}"), card.percentDone());
    write_to_lcd(message_buffer);
  }
}

/**
 * The Malyan LCD actually runs as a separate MCU on Serial 1.
 * This code's job is to siphon the weird curly-brace commands from
 * it and translate into gcode, which then gets injected into
 * the command queue where possible.
 */
extern "C" void lcd_uart_init();
void lcd_init() {
  sd_err_cnt=0;
  inbound_count = 0;
  //LCD_SERIAL.begin(500000);
  //lcd_uart_init();

  // Signal init
  write_to_lcd_P(PSTR("{SYS:STARTED}\r\n"));

  // send a version that says "unsupported"
  lcd_write_version();

  // No idea why it does this twice.
  write_to_lcd_P(PSTR("{SYS:STARTED}\r\n"));

  update_usb_status(true);
}

/**
 * Set an alert.
 */
void lcd_setalertstatusPGM(const char* message) {
  char message_buffer[MAX_CURLY_COMMAND];
  sprintf_P(message_buffer, PSTR("{E:%s}"), message);
  write_to_lcd(message_buffer);
}

#endif // Malyan LCD
