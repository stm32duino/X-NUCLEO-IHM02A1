/**
 ******************************************************************************
 * @file    X_NUCLEO_IHM02A1_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 October 2017
 * @brief   Arduino test application for the STMicroelectronics X-NUCLEO-IHM02A1
 *          Motor Control Expansion Board: control of 2 motors.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Helper header files. */
#include "SPI.h"

/* Expansion Board specific header files. */
#include "XNucleoIHM02A1.h"


/* Definitions ---------------------------------------------------------------*/

/* Number of movements per revolution. */
#define MPR_1 4

/* Number of steps. */
#define STEPS_1 (400 * 128)   /* 1 revolution given a 400 steps motor configured at 1/128 microstep mode. */
#define STEPS_2 (STEPS_1 * 2)

/* Delay in milliseconds. */
#define DELAY_1 1000
#define DELAY_2 2000
#define DELAY_3 5000

#define SerialPort Serial

/* Variables -----------------------------------------------------------------*/

/* Motor Control Expansion Board. */
XNucleoIHM02A1 *x_nucleo_ihm02a1;
SPIClass *dev_spi;
L6470 **motors;
int loops = 0;

/* Initialization parameters of the motors connected to the expansion board. */
L6470_init_t L6470_init[L6470DAISYCHAINSIZE] = {
    /* First Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    },

    /* Second Motor. */
    {
        9.0,                           /* Motor supply voltage in V. */
        400,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    }
};


/* setup ----------------------------------------------------------------------*/

void setup()
{
    /*----- Initialization. -----*/

    /* Initializing SPI bus. */
    dev_spi = new SPIClass(D11, D12, D3);
    SerialPort.begin(115200);

    /* Initializing Motor Control Expansion Board. */
    x_nucleo_ihm02a1 = new XNucleoIHM02A1(&L6470_init[0], &L6470_init[1], A4, A5, D4, A2, dev_spi);

    /* Building a list of motor control components. */
    motors = x_nucleo_ihm02a1->get_components();
}

/* loop ----------------------------------------------------------------------*/

void loop()
{
    loops++;

    /* Printing to the console. */
    SerialPort.print("Motor Control Application Example for 2 Motors. Loop: ");
    SerialPort.print(loops);
    SerialPort.print("\r\n\n");

    /* Set default microsteps. */
    L6470_init[0].step_sel = StepperMotor::STEP_MODE_1_128;
    if (!motors[0]->set_step_mode((StepperMotor::step_mode_t) L6470_init[0].step_sel)) {
        SerialPort.print("    Step Mode not allowed.\r\n");
    }

    /*----- Setting home and mark positions, getting positions, and going to positions. -----*/

    /* Printing to the console. */
    SerialPort.print("--> Setting home position.\r\n");

    /* Setting the home position. */
    motors[0]->set_home();

    /* Waiting. */
    delay(DELAY_1);

    /* Getting the current position. */
    int position = motors[0]->get_position();

    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Moving forward ");
    SerialPort.print(STEPS_1);
    SerialPort.print(" steps.\r\n");

    /* Moving. */
    motors[0]->move(StepperMotor::FWD, STEPS_1);

    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Printing to the console. */
    SerialPort.print("--> Marking the current position.\r\n");

    /* Marking the current position. */
    motors[0]->set_mark();

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Moving backward ");
    SerialPort.print(STEPS_2);
    SerialPort.print(" steps.\r\n");

    /* Moving. */
    motors[0]->move(StepperMotor::BWD, STEPS_2);

    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Waiting. */
    delay(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Going to marked position.\r\n");

    /* Going to marked position. */
    motors[0]->go_mark();
    
    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Waiting. */
    delay(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Going to home position.\r\n");

    /* Going to home position. */
    motors[0]->go_home();
    
    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Waiting. */
    delay(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Halving the microsteps.\r\n");

    /* Halving the microsteps. */
    L6470_init[0].step_sel = (L6470_init[0].step_sel > 0 ? L6470_init[0].step_sel -  1 : L6470_init[0].step_sel);
    if (!motors[0]->set_step_mode((StepperMotor::step_mode_t) L6470_init[0].step_sel)) {
        SerialPort.print("    Step Mode not allowed.\r\n");
    }

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Setting home position.\r\n");

    /* Setting the home position. */
    motors[0]->set_home();

    /* Waiting. */
    delay(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Waiting. */
    delay(DELAY_1);

    /* Printing to the console. */
    SerialPort.print("--> Moving forward ");
    SerialPort.print(STEPS_1);
    SerialPort.print(" steps.\r\n");

    /* Moving. */
    motors[0]->move(StepperMotor::FWD, STEPS_1);

    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    SerialPort.print("--> Getting the current position: ");
    SerialPort.print(position);
    SerialPort.print("\r\n");

    /* Printing to the console. */
    SerialPort.print("--> Marking the current position.\r\n");

    /* Marking the current position. */
    motors[0]->set_mark();

    /* Waiting. */
    delay(DELAY_2);


    /*----- Running together for a certain amount of time. -----*/

    /* Printing to the console. */
    SerialPort.print("--> Running together for ");
    SerialPort.print((DELAY_3 / 1000));
    SerialPort.print(" seconds.\r\n");

    /* Preparing each motor to perform a run at a specified speed. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_run(StepperMotor::BWD, 400);
    }

    /* Performing the action on each motor at the same time. */
    x_nucleo_ihm02a1->perform_prepared_actions();

    /* Waiting. */
    delay(DELAY_3);


    /*----- Increasing the speed while running. -----*/

    /* Preparing each motor to perform a run at a specified speed. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_get_speed();
    }

    /* Performing the action on each motor at the same time. */
    uint32_t* results = x_nucleo_ihm02a1->perform_prepared_actions();

    /* Printing to the console. */
    SerialPort.print("    Speed: M1 ");
    SerialPort.print(results[0]);
    SerialPort.print(", M2 ");
    SerialPort.print(results[1]);
    SerialPort.print(".\r\n");

    /* Printing to the console. */
    SerialPort.print("--> Doublig the speed while running again for ");
    SerialPort.print((DELAY_3 / 1000));
    SerialPort.print(" seconds.\r\n");

    /* Preparing each motor to perform a run at a specified speed. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_run(StepperMotor::BWD, results[m] << 1);
    }

    /* Performing the action on each motor at the same time. */
    results = x_nucleo_ihm02a1->perform_prepared_actions();

    /* Waiting. */
    delay(DELAY_3);

    /* Preparing each motor to perform a run at a specified speed. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_get_speed();
    }

    /* Performing the action on each motor at the same time. */
    results = x_nucleo_ihm02a1->perform_prepared_actions();

    /* Printing to the console. */
    SerialPort.print("    Speed: M1 ");
    SerialPort.print(results[0]);
    SerialPort.print(", M2 ");
    SerialPort.print(results[1]);
    SerialPort.print(".\r\n");

    /* Waiting. */
    delay(DELAY_1);


    /*----- Hard Stop. -----*/

    /* Printing to the console. */
    SerialPort.print("--> Hard Stop.\r\n");

    /* Preparing each motor to perform a hard stop. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_hard_stop();
    }

    /* Performing the action on each motor at the same time. */
    x_nucleo_ihm02a1->perform_prepared_actions();

    /* Waiting. */
    delay(DELAY_2);


    /*----- Doing a full revolution on each motor, one after the other. -----*/

    /* Printing to the console. */
    SerialPort.print("--> Doing a full revolution on each motor, one after the other.\r\n");

    /* Doing a full revolution on each motor, one after the other. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        for (int i = 0; i < MPR_1; i++) {
            /* Computing the number of steps. */
            int steps = (int) (((int) L6470_init[m].fullstepsperrevolution * pow(2.0f, L6470_init[m].step_sel)) / MPR_1);

            /* Moving. */
            motors[m]->move(StepperMotor::FWD, steps);
            
            /* Waiting while active. */
            motors[m]->wait_while_active();

            /* Waiting. */
            delay(DELAY_1);
        }
    }

    /* Waiting. */
    delay(DELAY_2);


    /*----- High Impedance State. -----*/

    /* Printing to the console. */
    SerialPort.print("--> High Impedance State.\r\n\r\n");

    /* Preparing each motor to set High Impedance State. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_hard_hiz();
    }

    /* Performing the action on each motor at the same time. */
    x_nucleo_ihm02a1->perform_prepared_actions();

    /* Waiting. */
    delay(DELAY_2);
}
