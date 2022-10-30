/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    IDX_CHAR_D,
    IDX_CHAR_VAL_D,

    IDX_CHAR_E,
    IDX_CHAR_VAL_E,

    IDX_CHAR_F,
    IDX_CHAR_VAL_F,
    IDX_CHAR_CFG_F,

    HRS_IDX_NB,
};


//电池电量
enum
{
    IDX_SVC_BATTERY,
    IDX_CHAR_BATTERY,
    IDX_CHAR_BATTERY_VAL,
    IDX_CHAR_BATTERY_CFG,
    IDX_BATTERY_MAX,
};