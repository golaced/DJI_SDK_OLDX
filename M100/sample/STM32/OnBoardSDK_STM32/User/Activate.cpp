/*! @file Activate.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Activation process for the STM32 example App.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "Activate.h"

extern CoreAPI defaultAPI;
extern CoreAPI *coreApi;
extern Flight flight;
extern FlightData flightData;

void
User_Activate ()
{
  static char key_buf[65] = "ba8c902e4a2cf1c883e126ba21288a9542ad15cdb9ce02e81715a206d4cfeb93"; /*"your app_key"*/

  ActivateData user_act_data;
  user_act_data.ID = 1032601; /*need your key in number like: 111000*/


  //! Change the version string to your platform/version as defined in DJI_Version.h
  user_act_data.version = versionM100_31;

  user_act_data.encKey = key_buf;

  coreApi->activate (&user_act_data);

}
