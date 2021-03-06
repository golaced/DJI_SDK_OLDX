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
  static char key_buf[65] = "4300bc4bbe2f9cf193bee93ab96d9f64dfb643946db18ce5a341d9bf734a75a4"; /*"your app_key"*/

  ActivateData user_act_data;
  user_act_data.ID = 1031836; /*need your key in number like: 111000*/


  //! Change the version string to your platform/version as defined in DJI_Version.h
  user_act_data.version = versionM100_31;

  user_act_data.encKey = key_buf;

  coreApi->activate (&user_act_data);

}
