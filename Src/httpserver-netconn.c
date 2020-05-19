/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/httpser-netconn.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-November-2015
  * @brief   Basic http server implementation using LwIP netconn API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "string.h"
#include "httpserver-netconn.h"
#include "cmsis_os.h"
#include "../webpages/index.h"
#include "temp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 4 )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32_t nPageHits = 0;
extern uint8_t out;
extern int Button;
extern uint8_t Test;
extern osSemaphoreId myBinarySem01Handle;
extern osSemaphoreId myBinarySem02Handle;
extern osSemaphoreId myBinarySem03Handle;
extern osSemaphoreId myBinarySem04Handle;
extern osSemaphoreId myBinarySem05Handle;
extern osSemaphoreId myBinarySem06Handle;
extern unsigned long t1;
extern TIM_HandleTypeDef htim4;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief serve tcp connection
  * @param conn: pointer on connection structure
  * @retval None
  */
void http_server_serve(struct netconn *conn)
{
  struct netbuf *inbuf;
  err_t recv_err;
  char* buf;
  u16_t buflen;

  /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
  recv_err = netconn_recv(conn, &inbuf);

  if (recv_err == ERR_OK)
  {
    if (netconn_err(conn) == ERR_OK)
    {
      netbuf_data(inbuf, (void**)&buf, &buflen);

      /* Is this an HTTP GET command? (only check the first 5 chars, since
      there are other formats for GET, and we're keeping it very simple )*/
      if ((buflen >=5) && (strncmp(buf, "GET /", 5) == 0))
      {

    	  if (strncmp((char const *)buf,"GET /index.html",15)==0) {
    		  netconn_write(conn, (const unsigned char*)index_html, index_html_len, NETCONN_NOCOPY);
    	  }
    	  if (strncmp((char const *)buf,"GET /led1", 9) == 0) {
    		  t1 =HAL_GetTick();
    		  __HAL_TIM_SetCounter(&htim4, 0);
    		  xSemaphoreGive(myBinarySem01Handle);
    	  }

    	  if (strncmp((char const *)buf,"GET /fun1", 9) == 0) {

    	     	    xSemaphoreGive(myBinarySem04Handle);
    	     	    }
    	  if (strncmp((char const *)buf,"GET /fun2", 9) == 0) {

    	      	     	    xSemaphoreGive(myBinarySem05Handle);
    	      	     	    }
    	  if (strncmp((char const *)buf,"GET /fun3", 9) == 0) {

    	      	      	     	    xSemaphoreGive(myBinarySem06Handle);
    	      	      	     	    }

    	  if (strncmp((char const *)buf,"GET /led2", 9) == 0) {
    		  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    		  xSemaphoreGive(myBinarySem02Handle);

    	  }
    	  if (strncmp((char const *)buf,"GET /led3", 9) == 0) {
    		  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    		  xSemaphoreGive(myBinarySem03Handle);
    	  }


    	  if (strncmp((char const *)buf,"GET /out1", 9) == 0) {
    		  	  out = out|Out0_Pin;
    	  }
    	  if (strncmp((char const *)buf,"GET /out2", 9) == 0) {
    		  	  out = out|Out1_Pin;
    	  }

    	  if (strncmp((char const *)buf,"GET /out3", 9) == 0) {
    		  	  out = out|Out2_Pin;
    	  }
    	  if (strncmp((char const *)buf,"GET /out4", 9) == 0) {
    		  	  out = out|Out3_Pin;
    	  }
    	  if (strncmp((char const *)buf,"GET /out5", 9) == 0) {
    	    	   out = out|Out4_Pin;
    	  }
    	  if (strncmp((char const *)buf,"GET /out6", 9) == 0) {
    	    	   out = out|Out5_Pin;
    	   }

    	   if (strncmp((char const *)buf,"GET /out7", 9) == 0) {
    	    	   out = out|Out6_Pin;
    	   }
    	   if (strncmp((char const *)buf,"GET /out8", 9) == 0) {
    	    	   out = out|Out7_Pin;
    	   }


    	   if (strncmp((char const *)buf,"GET /nout1", 9) == 0) {
    		   Test = ~(1 << 0);
    		   out &= ~(1 << 0);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout2", 9) == 0) {
    		   out &= ~(1 << 1);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout3", 9) == 0) {
    		   out &= ~(1 << 2);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout4", 9) == 0) {
    		   out &= ~(1 << 3);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout5", 9) == 0) {
    		   out &= ~(1 << 4);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout6", 9) == 0) {
    		   out &= ~(1 << 5);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout7", 9) == 0) {
    		   out &= ~(1 << 6);
    	   }
    	   if (strncmp((char const *)buf,"GET /nout8", 9) == 0) {
    		   out &= ~(1 << 7);
    	   }



    	  if (strncmp((char const *)buf,"GET /btn1", 9) == 0) {
    		  if(Button == 1)
    			  netconn_write(conn, (const unsigned char*)"ON", 2, NETCONN_NOCOPY);
    		  else
    			  netconn_write(conn, (const unsigned char*)"OFF", 3, NETCONN_NOCOPY);

    	  }





    	  // Uncomment Uncomment if ADC value is needed.
    	  /*
    	  if (strncmp((char const *)buf,"GET /adc", 8) == 0) {

    		  //buf = 25;
    		  //sprintf(buf, "%2.1f °C", getMCUTemperature());
    		  //sprintf(buf, "%2.1f °C",25.0); // Test without ADC
    		  netconn_write(conn, (const unsigned char*)buf, strlen(buf), NETCONN_NOCOPY);
    	  }
    	  */
      }
    }
  }
  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);

  /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
  netbuf_delete(inbuf);
}


/**
  * @brief  http server thread
  * @retval None
  */
static void http_server_netconn_thread()
{
  struct netconn *conn, *newconn;
  err_t err, accept_err;

  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);

  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 80);

    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);

      while(1)
      {
        /* accept any icoming connection */
        accept_err = netconn_accept(conn, &newconn);
        if(accept_err == ERR_OK)
        {
          /* serve connection */
          http_server_serve(newconn);

          /* delete connection */
          netconn_delete(newconn);
        }
      }
    }
  }
}

/**
  * @brief  Initialize the HTTP server (start its thread)
  * @param  none
  * @retval None
  */
void http_server_netconn_init()
{
  sys_thread_new("HTTP", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
}
