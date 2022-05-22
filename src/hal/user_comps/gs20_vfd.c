/*
    gs20_vfd. Modified the original GS2_vfd work to accomodate the replacement VFD now sold by Automation Direct
    Copyright (C) 2022 Chris Leschak
    
    gs2_vfd.
    Copyright (C) 2013 Sebastian Kuzminsky
    Copyright (C) 2009 John Thornton
    Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.

    Based on a work (test-modbus program, part of libmodbus) which is
    Copyright (C) 2001-2005 Stéphane Raimbault <stephane.raimbault@free.fr>

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation, version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301-1307  USA.


    This is a userspace program that interfaces the Automation Direct
    GS20 VFD's to the LinuxCNC HAL.

*/

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include "rtapi.h"
#include "hal.h"
#include <modbus.h>

/* Read GS20 Registers:  *note: (GG=param group) (nn=param number) Hex GGnn ex: P04.10 = 0x040A
	0-0x2100 = status word 1 
        High byte: Warning code
        Low Byte: Fault Code
	1-0x2101 = status word 2 
        Bit 1-0: AC motor drive operation status
                 00B: The drive stops
                 01B: The drive is decelerating
                 10B: The drive is in standby status
                 11B: The drive is operating
        Bit 2: 1 JOG command
        Bit 4-3: Operation direction
                 00B: FWD running
                 01B: From REV running to FWD running
                 10B: From FWD running to REV running
                 11B: REV running
        Bit 8: 1 Master frequency controlled by the communication interface
        Bit 9: 1 Master frequency controlled by hte analog / external terminal signal
        Bit 10: 1 Operation command controlled by the communication interface
        Bit 11: 1 Parameter locked
        Bit 12: 1 Enable to copy parameters from keypad
        Bit 15-13: Reserved for internal VFD use
	2-0x2102 = frequency command
	3-0x2103 = actual frequency
	4-0x2104 = output current
	5-0x2105 = DC bus voltage
	6-0x2106 = actual output voltage
	**7-0x2107 = Current step for the multi-step speed operation ** changed from GS2
	**8-0x2108 = Reserved for internal VFD use
    9-0x2109 = Digital Input Counter value (dont need)
	10-0x210A = Output power factor angle (xxx.x) ** changed from GS2
	11-0x210B = Output torque (xxx.x %)
	12-0x210C = Actual motor speed (xxxxx rpm)
    **13-0x210D = Reserved for internal VFD use ** new but not used
    **14-0x210E = Reserved for internal VFD use ** new but not used
    **15-0x210F = Power output (x.xxx kW) ** new?
	total of 12 registers		
    NOTE: Interested in possibly adding the following individual read registers
    0x220E = IGBT temperature of the power module (xxx.x deg C)
    0x221B = DC bus voltage ripples (xxx.x V)
    0x221D = Magnetic field area of the synchronous motor */
#define START_REGISTER_R	0x2100
#define NUM_REGISTERS_R		16
/* write registers:
    0x2000 = 16 bit word
            bit 1–0    00B: No function *Replace GS2 0x91B
                       01B: Stop
                       10B: Run
                       11B: JOG + RUN
            bit 3–2 Reserved
            bit 5–4    00B: No function *Replace GS2 0x91C
                       01B: FWD
                       10B: REV
                       11B: Change direction
            bit 7–6    00B: 1st accel / decel
                       01B: 2nd accel / decel
                       10B: 3rd accel / decel
                       11B: 4th accel / decel
            bit 11–8 0000B: Master speed
                     0001B: 1st step speed frequency
                     0010B: 2nd step speed frequency
                     0011B: 3rd step speed frequency
                     0100B: 4th step speed frequency
                     0101B: 5th step speed frequency
                     0110B: 6th step speed frequency
                     0111B: 7th step speed frequency
                     1000B: 8th step speed frequency
                     1001B: 9th step speed frequency
                     1010B: 10th step speed frequency
                     1011B: 11th step speed frequency
                     1100B: 12th step speed frequency
                     1101B: 13th step speed frequency
                     1110B: 14th step speed frequency
                     1111B: 15th step speed frequency
            bit 12 (1) Enable bit 06–11 function
            bit 14–13  00B: No function
                       01B: Operated by the digital keypad
                       10B: Operated by Pr00-21 setting
                       11B: Change the operation source
            bit 15 Reserved
    0x2001 = Frequency command (xxx.xx Hz) *Eqv to GS2 0x091A Speed ref

	0x2002 = 16 bit word
             bit 0 (1) EF (External Fault) ON *Eqv to GS2 0x91D
             bit 1 (1) Reset command *Eqv to GS2 0x91E
             bit 2 (1) BB ON
             bit 4–3   Reserved for internal VFD use
             bit 5 (1) Enable fire mode
             bit 15–6  Reserved for internal VFD use
	total of 3 registers */
#define START_REGISTER_W	0x2000
#define NUM_REGISTERS_W		3
//#define MODBUS_MAX_WRITE_BITS    2

#define GS20_REG_STOP_METHOD                            0x0016
#define GS20_REG_STOP_METHOD__RAMP_TO_STOP              0
#define GS20_REG_STOP_METHOD__COAST_TO_STOP             1

#define GS20_REG_ACCELERATION_TIME_1                    0x010C

#define GS20_REG_DECELERATION_TIME_1                    0x010D

#define GS20_REG_OVER_VOLTAGE_STALL_PREVENTION           0x0601
#define GS20_REG_OVER_VOLTAGE_STALL_PREVENTION__ENABLE   380.0 //120v-230v 0.0-390.0vdc  
#define GS20_REG_OVER_VOLTAGE_STALL_PREVENTION__DISABLE  0.0


/* modbus slave data struct */
typedef struct {
	int slave;		/* slave address */
	int read_reg_start;	/* starting read register number */
	int read_reg_count;	/* number of registers to read */
	int write_reg_start;	/* starting write register number */
	int write_reg_count;	/* number of registers to write */
} slavedata_t;

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!this needs edit below
/* HAL data struct */
typedef struct {
  hal_s32_t	    *stat1;	   	  // rd 0 status words from the VFD.  Maybe split these out sometime
  hal_s32_t  	*stat2;       // rd 1
  hal_float_t	*freq_cmd;	  // rd 2 frequency command *where you want to be
  hal_float_t	*freq_out;	  // rd 3 actual output frequency *where you are
  hal_float_t	*curr_out;	  // rd 4 output current
  hal_float_t	*DCBusV;	  // rd 5 buss volgate
  hal_float_t	*outV;        // rd 6 output voltage
  hal_float_t	*power_factor;// rd 10 output power factor angle
  hal_float_t	*load_pct;    // rd 11 output torque %
  hal_float_t	*RPM;         // rd 12
  //hal_float_t	*scale_freq;  // rd reg? / reg? = scaled freq  
  //hal_s32_t	    *FW_Rev;      // rd 0x0006 Firmware Revision already brought in..
  hal_s32_t	    errorcount;   // Calculated if null return value
  hal_float_t	looptime;     // Monitors loop time of modbus cycle
  hal_float_t	speed_tolerance; // no idea
  hal_s32_t 	retval;         // retrieved value variable
  hal_bit_t		*at_speed;		// calculated when drive freq_cmd == freq_out and running
  hal_bit_t		*is_stopped;	// calculated when drive freq out is 0
  hal_float_t	*speed_command; // wr speed command input 0x2001
  hal_float_t	motor_hz;		// speeds are scaled in Hz, not RPM
  hal_float_t	motor_RPM;		// nameplate RPM at default Hz
  hal_bit_t 	*spindle_on;	// wr spindle 10=on, 01=off  0x2000 bit 1-0
  hal_bit_t 	*spindle_fwd;	// wr direction, 01B=fwd, 0x2000 bit 5-4
  hal_bit_t     *spindle_rev;	// wr direction, 10B-rev, 0x2000 bit 5-4
  hal_bit_t 	*err_reset;		// reset errors, 1-reset, 0x2002 bit 1
  hal_s32_t     ack_delay;		// number of read/writes before checking at-speed
  hal_bit_t 	old_run;		// so we can detect changes in the run state
  hal_bit_t 	old_dir;        // so we can detect changes in direction
  hal_bit_t 	old_err_reset;  // so we can detect changes in rest output state
  hal_bit_t     *ena_gs20comp;  // wr enable pin 1-ena, 0-dis bit 12
  hal_bit_t     *isInitialized; // initialized status pin
} haldata_t;

static int done;
char *modname = "gs20_vfd";

static struct option long_options[] = {
    {"bits", 1, 0, 'b'},
    {"device", 1, 0, 'd'},
    {"debug", 0, 0, 'g'},
    {"help", 0, 0, 'h'},
    {"name", 1, 0, 'n'},
    {"parity", 1, 0, 'p'},
    {"rate", 1, 0, 'r'},
    {"stopbits", 1, 0, 's'},
    {"target", 1, 0, 't'},
    {"verbose", 0, 0, 'v'},
    {"accel-seconds", required_argument, NULL, 'A'},
    {"decel-seconds", required_argument, NULL, 'D'},
    {"braking-resistor", no_argument, NULL, 'R'},
    {"disable", no_argument, NULL, 'X'},
    {0,0,0,0}
};

static char *option_string = "gb:d:hn:p:r:s:t:vA:D:RX";

static char *bitstrings[] = {"5", "6", "7", "8", NULL};

// The old libmodbus (v2?) used strings to indicate parity, the new one
// (v3.0.1) uses chars.  The gs2_vfd driver gets the string indicating the
// parity to use from the command line, and I don't want to change the
// command-line usage.  The command-line argument string must match an
// entry in paritystrings, and the index of the matching string is used as
// the index to the parity character for the new libmodbus.
static char *paritystrings[] = {"even", "odd", "none", NULL};
static char paritychars[] = {'E', 'O', 'N'};

static char *ratestrings[] = {"110", "300", "600", "1200", "2400", "4800", "9600",
    "19200", "38400", "57600", "115200", NULL};
static char *stopstrings[] = {"1", "2", NULL};

static void quit(int sig) {
    done = 1;
}

static int comm_delay = 0; // JET delay counter for at-speed

int match_string(char *string, char **matches) {
    int len, which, match;
    which=0;
    match=-1;
    if ((matches==NULL) || (string==NULL)) return -1;
    len = strlen(string);
    while (matches[which] != NULL) {
        if ((!strncmp(string, matches[which], len)) && (len <= strlen(matches[which]))) {
            if (match>=0) return -1;        // multiple matches
            match=which;
        }
        ++which;
    }
    return match;
}


int gs20_set_accel_time(modbus_t *mb_ctx, float accel_time) {
    int data = accel_time * 10;
    int r;

    r = modbus_write_register(mb_ctx, GS20_REG_ACCELERATION_TIME_1, data);
    if (r != 1) {
        // Retry, test system always fails first communication
        r = modbus_write_register(mb_ctx, GS20_REG_ACCELERATION_TIME_1, data);
        if (r != 1) {
            fprintf(
                stderr,
                "failed to set register P0x%04x to 0x%04x (%d): %s\n",
                GS20_REG_ACCELERATION_TIME_1,
                data, data,
                strerror(errno)
            );
            return -1;
        }
    }

    return 0;
}


int gs20_set_decel_time(modbus_t *mb_ctx, float decel_time) {
    int data;
    int stop_method;
    int r;

    if (decel_time == 0.0) {
        stop_method = GS20_REG_STOP_METHOD__COAST_TO_STOP;
        decel_time = 20.0;
    } else {
        stop_method = GS20_REG_STOP_METHOD__RAMP_TO_STOP;
    }
    r = modbus_write_register(mb_ctx, GS20_REG_STOP_METHOD, stop_method);
    if (r != 1) {
        fprintf(
            stderr,
            "failed to set register P0x%04x to 0x%04x: %s\n",
            GS20_REG_STOP_METHOD,
            stop_method,
            strerror(errno)
        );
        return -1;
    }

    data = decel_time * 10;
    r = modbus_write_register(mb_ctx, GS20_REG_DECELERATION_TIME_1, data);
    if (r != 1) {
        fprintf(
            stderr,
            "failed to set register P0x%04x to 0x%04x (%d): %s\n",
            GS20_REG_DECELERATION_TIME_1,
            data, data,
            strerror(errno)
        );
        return -1;
    }

    return 0;
}


int gs20_set_braking_resistor(modbus_t *mb_ctx, int braking_resistor) {
    int data;
    int r;

    if (braking_resistor) {
        data = GS20_REG_OVER_VOLTAGE_STALL_PREVENTION__DISABLE;
    } else {
        data = GS20_REG_OVER_VOLTAGE_STALL_PREVENTION__ENABLE;
    }
    r = modbus_write_register(
        mb_ctx,
        GS20_REG_OVER_VOLTAGE_STALL_PREVENTION,
        data
    );
    if (r != 1) {
        fprintf(
            stderr,
            "failed to set register P0x%04x to 0x%04x: %s\n",
            GS20_REG_OVER_VOLTAGE_STALL_PREVENTION,
            data,
            strerror(errno)
        );
        return -1;
    }

    return 0;
}


typedef struct {
    uint8_t param_group, param_number;
    const char *name;
} gs20_reg;
/* 
struct array[n] = param_group, param_number, "*name"
*/
gs20_reg gs20_register[] = {
    { 0x00, 0x00, "GS20 Model ID Voltage, Phase, Hp" },
    { 0x00, 0x01, "GS20 Model rated current" },
    { 0x00, 0x02, "Restore to Default" },
    { 0x00, 0x03, "Start-up display selection" },
    { 0x00, 0x04, "User Display" },
    { 0x00, 0x05, "Coefficient Gain in Actual Output Freq Display" },
    { 0x00, 0x06, "Firmware version" }, 
    { 0x00, 0x0A, "Control Method" },
    { 0x00, 0x0B, "Speed (Velocity) Control Mode" },
    { 0x00, 0x10, "Torque duty selection" },
    { 0x00, 0x11, "Carrier frequency" },
    { 0x00, 0x14, "Master frequency command source (REMOTE, AUTO)" },
    { 0x00, 0x15, "Operation command source (REMOTE, AUTO)" },
    { 0x00, 0x16, "Stop method" },
    { 0x00, 0x17, "Motor direction control" },
    { 0x00, 0x1E, "Master frequency command source (HAND, LOCAL)" },
    { 0x00, 0x1F, "Operation command source (HAND, LOCAL)" },
    { 0x01, 0x00, "Maximum operation frequency" },
    { 0x01, 0x01, "Motor 1 Frequency Base" },
    { 0x01, 0x02, "Motor 1 Nameplate rated voltage" },
    { 0x01, 0x03, "Motor 1 Mid-point frequency" },
    { 0x01, 0x04, "Motor 1 Mid-point voltage" },
    { 0x01, 0x05, "Motor 1 Mid-point frequency 2" },
    { 0x01, 0x06, "Motor 1 Mid-Point voltage 2" },
    { 0x01, 0x07, "Motor 1 Min output frequency" },
    { 0x01, 0x08, "Motor 1 Min output voltage" },
    { 0x01, 0x09, "Start-up frequency" },
    { 0x01, 0x0A, "Output frequency upper limit" },
    { 0x01, 0x0B, "Output frequency lower limit" },
    { 0x01, 0x0C, "Acceleration time 1" },
    { 0x01, 0x0D, "Deceleration time 1" },
    { 0x00, 0x00, NULL }  // NULL name mean "end of list"
};

/*
returnValue myFunction(Argument) {
    Method Body
    }
void means returns no value.   */
void gs20_show_config(modbus_t *mb_ctx) { 
    gs20_reg *reg;  //*reg is a pointer variable of the gs20_reg struct type
    int r;
/*
for (initializationStatement; testExpression; updateStatement)
As long as the name variable does not return NULL, reg will add 1 to its value and rerun
*/
    for (reg = &gs20_register[0]; reg->name != NULL; reg ++) {
        int address;
        uint16_t data;

        /*
        address = current list index paramater group shifted left 8 *bitwise or* non shifted index parameter (bitwise or - compares both and if either has a 1 then its 1 
            Because both original addresses were 8 bit, this will create a 16 bit address from param group.number)
        */
        address = (reg->param_group << 8) | reg->param_number;

        r = modbus_read_registers(mb_ctx, address, 1, &data);
        if (r != 1) {
            fprintf(
                stderr,
                "failed to read register P%d.%02d (%s)\n",
                reg->param_group,
                reg->param_number,
                reg->name
            );
            return;
        }
        printf(
            "P%d.%02d %s: 0x%04x (%d)\n",
            reg->param_group,
            reg->param_number,
            reg->name,
            data,
            data
        );
    }
}
 
 //int modbus_write_register(modbus_t *ctx, int addr, int value)
 //{
 //    return write_single(ctx, _FC_WRITE_SINGLE_REGISTER, addr, value);
 //}
int write_data(modbus_t *mb_ctx, slavedata_t *slavedata, haldata_t *haldata) {
//  int write_data[MAX_WRITE_REGS];
    int retval;
    hal_float_t hzcalc;
    uint16_t s_run_stop; //Spindle Run / Stop cmd
    uint16_t s_dir; //Spindle Fwd / Rev cmd
    uint16_t reg_0x2000_val; //Complete 16 bit register

    if (haldata->motor_hz<10)
        haldata->motor_hz = 60;
    if ((haldata->motor_RPM < 600) || (haldata->motor_RPM > 5000))
        haldata->motor_RPM = 1800;
    hzcalc = haldata->motor_hz/haldata->motor_RPM;
    //************ Write Register 2 Value **************
    retval = modbus_write_register(
        mb_ctx,
        slavedata->write_reg_start+1,
        abs((int)(*(haldata->speed_command)*hzcalc*10))
    );
    /*                  ****  These are all bits from the same register  ****
    Code should accumulate the state of each condition, then write the entire reg value together
    ************ Write Register 1 Value ***************
    */
    if ((*(haldata->spindle_on) != haldata->old_run) || (*(haldata->spindle_fwd) != haldata->old_dir)) {
        if (*haldata->spindle_on){
            s_run_stop = 2;
            //modbus_write_register(mb_ctx, slavedata->write_reg_start, 0000000000000010); //2
            comm_delay=0; //Not sure if this is needed
        }
        else
            s_run_stop = 1;
            //modbus_write_register(mb_ctx, slavedata->write_reg_start, 0000000000000001); //1
        //haldata->old_run = *(haldata->spindle_on);
    //}
    //if (*(haldata->spindle_fwd) != haldata->old_dir) {
        if (*haldata->spindle_fwd){
            s_dir = 8;
            //modbus_write_register(mb_ctx, slavedata->write_reg_start, 0000000000001000); //8
        }
        else
            s_dir = 16;
            //modbus_write_register(mb_ctx, slavedata->write_reg_start, 0000000000010000); //16
        reg_0x2000_val = s_run_stop + s_dir; // Add up the results of the individual bit values of each command and store in a variable
        modbus_write_register(mb_ctx, slavedata->write_reg_start, reg_0x2000_val); // in one single write action, write the state of all 16 bits
        haldata->old_run = *(haldata->spindle_on); // update the hal pin status
        haldata->old_dir = *(haldata->spindle_fwd); //update the hal pin status        
    }
    if (*(haldata->spindle_fwd) || !(*(haldata->spindle_on)))  // JET turn on and off rev based on the status of fwd
        *(haldata->spindle_rev) = 0;
    if (!(*haldata->spindle_fwd) && *(haldata->spindle_on))
        *(haldata->spindle_rev) = 1;

    if (*(haldata->err_reset) != haldata->old_err_reset) {
        if (*(haldata->err_reset))
            modbus_write_register(mb_ctx, slavedata->write_reg_start+2, 2); // 0000000000000010
        else
            modbus_write_register(mb_ctx, slavedata->write_reg_start+2, 0); // 0000000000000000
        haldata->old_err_reset = *(haldata->err_reset);
    }
    if (comm_delay < haldata->ack_delay){ // JET allow time for communications between drive and EMC
        comm_delay++;
    }
    if ((*haldata->spindle_on) && comm_delay == haldata->ack_delay){ // JET test for up to speed
        if ((*(haldata->freq_cmd))==(*(haldata->freq_out)))
            *(haldata->at_speed) = 1;
    }
    if (*(haldata->spindle_on)==0){ // JET reset at-speed
        *(haldata->at_speed) = 0;
    }
    haldata->retval = retval;
    return retval;
}

void usage(int argc, char **argv) {
    printf("Usage:  %s [options]\n", argv[0]);
    printf(
    "This is a userspace HAL program, typically loaded using the halcmd \"loadusr\" command:\n"
    "    loadusr gs20_vfd\n"
    "There are several command-line options.  Options that have a set list of possible values may\n"
    "    be set by using any number of characters that are unique.  For example, --rate 5 will use\n"
    "    a baud rate of 57600, since no other available baud rates start with \"5\"\n"
    "-b or --bits <n> (default 8)\n"
    "    Set number of data bits to <n>, where n must be from 5 to 8 inclusive\n"
    "-d or --device <path> (default /dev/ttyS0)\n"
    "    Set the name of the serial device node to use\n"
    "-v or --verbose\n"
    "    Turn on verbose mode.\n"
    "-g or --debug\n"
    "    Turn on debug mode.  This will cause all modbus messages to be\n"
    "    printed in hex on the terminal.\n"
    "-n or --name <string> (default gs20_vfd)\n"
    "    Set the name of the HAL module.  The HAL comp name will be set to <string>, and all pin\n"
    "    and parameter names will begin with <string>.\n"
    "-p or --parity {even,odd,none} (default odd)\n"
    "    Set serial parity to even, odd, or none.\n"
    "-r or --rate <n> (default 38400)\n"
    "    Set baud rate to <n>.  It is an error if the rate is not one of the following:\n"
    "    110, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200\n"
    "-s or --stopbits {1,2} (default 1)\n"
    "    Set serial stop bits to 1 or 2\n"
    "-t or --target <n> (default 1)\n"
    "    Set MODBUS target (slave) number.  This must match the device number you set on the GS2.\n"
    "-A, --accel-seconds <n>\n"
    "    (default 10.0) Seconds to accelerate the spindle from 0 to Max RPM.\n"
    "-D, --decel-seconds <n>\n"
    "    (default 0.0) Seconds to decelerate the spindle from Max RPM to 0.\n"
    "    If set to 0.0 the spindle will be allowed to coast to a stop without\n"
    "    controlled deceleration.\n"
    "-R, --braking-resistor\n"
    "    This argument should be used when a braking resistor is installed on the\n" //edit this
    "    GS20 VFD (see Appendix A of the GS20 manual).  It disables deceleration\n"
    "    over-voltage stall prevention (see GS20 modbus Parameter 6.01), allowing\n"
    "    the VFD to keep braking even in situations where the motor is regenerating\n"
    "    high voltage.  The regenerated voltage gets safely dumped into the\n"
    "    braking resistor.\n"
    "-X, --disable\n"
    "    Set this flag to disable the control by default (sets default value of 'enable' pin to 0)"
    );
}
int read_data(modbus_t *mb_ctx, slavedata_t *slavedata, haldata_t *hal_data_block) {
    uint16_t receive_data[MODBUS_MAX_READ_REGISTERS];	/* a little padding in there */
    int retval;

    /* can't do anything with a null HAL data block */
    if (hal_data_block == NULL)
        return -1;
    /* but we can signal an error if the other params are null */
    if ((mb_ctx==NULL) || (slavedata == NULL)) {
        hal_data_block->errorcount++;
        return -1;
    }
    retval = modbus_read_registers(mb_ctx, slavedata->read_reg_start,
                                slavedata->read_reg_count, receive_data);
    if (retval==slavedata->read_reg_count) {
        retval = 0;
        hal_data_block->retval = retval;
        *(hal_data_block->stat1) = receive_data[0];
        *(hal_data_block->stat2) = receive_data[1];  //*(hal_data_block->FW_Rev) = receive_data[1];//
        *(hal_data_block->freq_cmd) = receive_data[2] * 0.1;
        *(hal_data_block->freq_out) = receive_data[3] * 0.1;
        if (receive_data[3]==0) {	// JET if freq out is 0 then the drive is stopped
            *(hal_data_block->is_stopped) = 1;
        } else {
            *(hal_data_block->is_stopped) = 0;
        }
        *(hal_data_block->curr_out) = receive_data[4] * 0.1;
        *(hal_data_block->DCBusV) = receive_data[5] * 0.1;
        *(hal_data_block->outV) = receive_data[6] * 0.1;
        *(hal_data_block->power_factor) = receive_data[10];
        *(hal_data_block->load_pct) = receive_data[11] * 0.1;        
        *(hal_data_block->RPM) = receive_data[12];
    } else {
        hal_data_block->retval = retval;
        hal_data_block->errorcount++;
        retval = -1;
    }
    return retval;
}

int main(int argc, char **argv)
{
    int retval = 0;
    modbus_t *mb_ctx;
    haldata_t *haldata;
    slavedata_t slavedata;
    int slave;
    int hal_comp_id;
    struct timespec loop_timespec, remaining;
    int baud, bits, stopbits, verbose, debug;
    char *device, *endarg;
    char parity;
    int opt;
    int argindex, argvalue;
    int enabled;

    float accel_time = 10.0;
    float decel_time = 0.0;  // this means: coast to a stop, don't try to control deceleration time
    int braking_resistor = 0;


    done = 0;

    // assume that nothing is specified on the command line
    baud = 38400;
    bits = 8;
    stopbits = 1;
    debug = 0;
    verbose = 0;
    device = "/dev/ttyS0";
    parity = 'O';
    enabled = 1;

    /* slave / register info */
    slave = 1;
    slavedata.read_reg_start = START_REGISTER_R;
    slavedata.read_reg_count = NUM_REGISTERS_R;
    slavedata.write_reg_start = START_REGISTER_W;
    slavedata.write_reg_count = NUM_REGISTERS_W;

    // process command line options
    while ((opt=getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch(opt) {
            case 'X':  // disable by default on startup
                enabled = 0;
                break;
            case 'b':   // serial data bits, probably should be 8 (and defaults to 8)
                argindex=match_string(optarg, bitstrings);
                if (argindex<0) {
                    printf("gs20_vfd: ERROR: invalid number of bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                bits = atoi(bitstrings[argindex]);
                break;
            case 'd':   // device name, default /dev/ttyS0
                // could check the device name here, but we'll leave it to the library open
                if (strlen(optarg) > FILENAME_MAX) {
                    printf("gs20_vfd: ERROR: device node name is too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                device = strdup(optarg);
                break;
            case 'g':
                debug = 1;
                break;
            case 'v':
                verbose = 1;
                break;
            case 'n':   // module base name
                if (strlen(optarg) > HAL_NAME_LEN-20) {
                    printf("gs20_vfd: ERROR: HAL module name too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                modname = strdup(optarg);
                break;
            case 'p':   // parity, should be a string like "even", "odd", or "none"
                argindex=match_string(optarg, paritystrings);
                if (argindex<0) {
                    printf("gs20_vfd: ERROR: invalid parity: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                parity = paritychars[argindex];
                break;
            case 'r':   // Baud rate, 38400 default
                argindex=match_string(optarg, ratestrings);
                if (argindex<0) {
                    printf("gs20_vfd: ERROR: invalid baud rate: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                baud = atoi(ratestrings[argindex]);
                break;
            case 's':   // stop bits, defaults to 1
                argindex=match_string(optarg, stopstrings);
                if (argindex<0) {
                    printf("gs20_vfd: ERROR: invalid number of stop bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                stopbits = atoi(stopstrings[argindex]);
                break;
            case 't':   // target number (MODBUS ID), default 1
                argvalue = strtol(optarg, &endarg, 10);
                if ((*endarg != '\0') || (argvalue < 1) || (argvalue > 254)) {
                    printf("gs20_vfd: ERROR: invalid slave number: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                slave = argvalue;
                break;
            case 'A':
                accel_time = strtof(optarg, &endarg);
                if (*endarg != '\0') {
                
                    printf("gs20_vfd: ERROR: invalid acceleration time: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                break;
            case 'D':
                decel_time = strtof(optarg, &endarg);
                if (*endarg != '\0') {
                    printf("gs20_vfd: ERROR: invalid deceleration time: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                break;
            case 'R':
                braking_resistor = 1;
                break;
            case 'h':
            default:
                usage(argc, argv);
                exit(0);
                break;
        }
    }

    printf("%s: device='%s', baud=%d, parity='%c', bits=%d, stopbits=%d, address=%d, enabled=%d\n",
           modname, device, baud, parity, bits, stopbits, slave, enabled);
    /* point TERM and INT signals at our quit function */
    /* if a signal is received between here and the main loop, it should prevent
            some initialization from happening */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Assume 38.4k O-8-1 serial settings, device 1 */
    mb_ctx = modbus_new_rtu(device, baud, parity, bits, stopbits);
    if (mb_ctx == NULL) {
        printf("%s: ERROR: couldn't open modbus serial device: %s\n", modname, modbus_strerror(errno));
        goto out_noclose;
    }

    /* the open has got to work, or we're out of business */
    if (((retval = modbus_connect(mb_ctx))!=0) || done) {
        printf("%s: ERROR: couldn't open serial device: %s\n", modname, modbus_strerror(errno));
        goto out_noclose;
    }

    modbus_set_debug(mb_ctx, debug);

    modbus_set_slave(mb_ctx, slave);

    // show the gs20 vfd configuration
    if (verbose) {
        gs20_show_config(mb_ctx);
    }

    /* create HAL component */
    hal_comp_id = hal_init(modname);
    if ((hal_comp_id < 0) || done) {
        printf("%s: ERROR: hal_init failed\n", modname);
        retval = hal_comp_id;
        goto out_close;
    }

    /* grab some shmem to store the HAL data in */
    haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if ((haldata == 0) || done) {
        printf("%s: ERROR: unable to allocate shared memory\n", modname);
        retval = -1;
        goto out_close;
    }

    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->stat1), hal_comp_id, "%s.status-1", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->stat2), hal_comp_id, "%s.status-2", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->freq_cmd), hal_comp_id, "%s.frequency-command", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->freq_out), hal_comp_id, "%s.frequency-out", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->curr_out), hal_comp_id, "%s.output-current", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->DCBusV), hal_comp_id, "%s.DC-bus-volts", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->outV), hal_comp_id, "%s.output-voltage", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->RPM), hal_comp_id, "%s.motor-RPM", modname);
    if (retval!=0) goto out_closeHAL;
    //retval = hal_pin_float_newf(HAL_OUT, &(haldata->scale_freq), hal_comp_id, "%s.scale-frequency", modname);
    //if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->power_factor), hal_comp_id, "%s.power-factor", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->load_pct), hal_comp_id, "%s.load-percentage", modname);
    if (retval!=0) goto out_closeHAL;
    //retval = hal_pin_s32_newf(HAL_OUT, &(haldata->FW_Rev), hal_comp_id, "%s.firmware-revision", modname);
    //if (retval!=0) goto out_closeHAL;
    retval = hal_param_s32_newf(HAL_RW, &(haldata->errorcount), hal_comp_id, "%s.error-count", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->looptime), hal_comp_id, "%s.loop-time", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_s32_newf(HAL_RW, &(haldata->retval), hal_comp_id, "%s.retval", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->at_speed), hal_comp_id, "%s.at-speed", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->is_stopped), hal_comp_id, "%s.is-stopped", modname); // JET
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->speed_command), hal_comp_id, "%s.speed-command", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_on), hal_comp_id, "%s.spindle-on", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_fwd), hal_comp_id, "%s.spindle-fwd", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_rev), hal_comp_id, "%s.spindle-rev", modname); //JET
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->err_reset), hal_comp_id, "%s.err-reset", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->speed_tolerance), hal_comp_id, "%s.tolerance", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->motor_hz), hal_comp_id, "%s.nameplate-HZ", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->motor_RPM), hal_comp_id, "%s.nameplate-RPM", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_s32_newf(HAL_RW, &(haldata->ack_delay), hal_comp_id, "%s.ack-delay", modname);
    if (retval!=0) goto out_closeHAL;
    /* define run (enable) pin and isInitialized */
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->ena_gs20comp), hal_comp_id, "%s.enable", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->isInitialized), hal_comp_id, "%s.initialized", modname);
    if (retval!=0) goto out_closeHAL;

    /* make default data match what we expect to use */
    *(haldata->stat1) = 0;
    *(haldata->stat2) = 0;
    *(haldata->freq_cmd) = 0;
    *(haldata->freq_out) = 0;
    *(haldata->curr_out) = 0;
    *(haldata->DCBusV) = 0;
    *(haldata->outV) = 0;
    *(haldata->RPM) = 0;
    //*(haldata->scale_freq) = 0;
    *(haldata->power_factor) = 0;
    *(haldata->load_pct) = 0;
    //*(haldata->FW_Rev) = 0;
    haldata->errorcount = 0;
    haldata->looptime = 0.1;
    haldata->motor_RPM = 1730;
    haldata->motor_hz = 60;
    haldata->speed_tolerance = 0.01;
    haldata->ack_delay = 2;
    *(haldata->err_reset) = 0;
    *(haldata->spindle_on) = 0;
    *(haldata->spindle_fwd) = 1;
    *(haldata->spindle_rev) = 0;
    haldata->old_run = -1;		// make sure the initial value gets output
    haldata->old_dir = -1;
    haldata->old_err_reset = -1;
    *(haldata->ena_gs20comp) = enabled;  // command line override, defaults to "enabled" for compatibility
    *(haldata->isInitialized) = 0;

    // Activate HAL component
    hal_ready(hal_comp_id);



    /* here's the meat of the program.  loop until done (which may be never) */
    while (done==0) {

        /* don't want to scan too fast, and shouldn't delay more than a few seconds */
        if (haldata->looptime < 0.001) haldata->looptime = 0.001;
        if (haldata->looptime > 2.0) haldata->looptime = 2.0;
        loop_timespec.tv_sec = (time_t)(haldata->looptime);
        loop_timespec.tv_nsec = (long)((haldata->looptime - loop_timespec.tv_sec) * 1000000000l);
        nanosleep(&loop_timespec, &remaining);

        if(*(haldata->ena_gs20comp) == 0) {
             // Component not enabled, so do nothing and force uninitialized state
             if (*(haldata->isInitialized)) {
                *(haldata->spindle_on) = 0;
                // need to write to vfd in case we are here when it is being disabled
                write_data(mb_ctx, &slavedata, haldata);
                // debug printf below
                // printf("GS20: Disabling\n");
            }
            *(haldata->isInitialized) = 0;
        } else if (!*(haldata->isInitialized)) {
            // Initialize: configure the gs2 vfd based on command-line arguments
            if (gs20_set_accel_time(mb_ctx, accel_time) != 0) {
                continue;
            }
            if (gs20_set_decel_time(mb_ctx, decel_time) != 0) {
                continue;
            }
            if (gs20_set_braking_resistor(mb_ctx, braking_resistor) != 0) {
                continue;
            }
            // debug printf below
            // printf("GS20: Initialized\n");
            *(haldata->isInitialized) = 1;
        } else {
            // Enabled and initialized, so do read/write of Modbus
            read_data(mb_ctx, &slavedata, haldata);
            write_data(mb_ctx, &slavedata, haldata);
        }
    }

    retval = 0;	/* if we get here, then everything is fine, so just clean up and exit */
out_closeHAL:
    hal_exit(hal_comp_id);
out_close:
    modbus_close(mb_ctx);
    modbus_free(mb_ctx);
out_noclose:
    return retval;
}