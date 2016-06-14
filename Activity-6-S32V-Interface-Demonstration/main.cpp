/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#ifndef __STANDALONE__
  #include <signal.h>
#include <opencv2/opencv.hpp>  	//added
#endif // #ifdef __STANDALONE__
#include <string.h>
#include "frame_input_patterngen.h"
#include "frame_input_isp.h"

#ifdef __STANDALONE__
  #include "frame_output_dcu.h"
  #define CHNL_CNT io::IO_DATA_CH3
#else // #ifdef __STANDALONE__
  #include "frame_output_v234fb.h"
//#include "frame_output_dcu.h"	//added
//#include <time.h>		//added
#include "time_measure.h"	//added
  #define CHNL_CNT io::IO_DATA_CH3
#endif // else from #ifdef __STANDALONE__  

#include "seq_public.h"

//***************************************************************************

// Possible to set input resolution (must be supported by the DCU)
#define WIDTH           1280 ///< width of DDR buffer in pixels
#define HEIGHT          720 ///< height of DDR buffer in pixels
#define DDR_BUFFER_CNT  3    ///< number of DDR buffers per ISP stream

//***************************************************************************

/************************************************************************/
/** Allocates contiguous DDR buffers for one ISP stream.
  * 
  * \param  appFbsVirtual array of virtual buffer pointers to be filled
  * \param  apFbsPhysical array of buffer physical addresses to be filled
  * 
  * \return 0 if all OK
  *         < 0 otherwise
  ************************************************************************/
int32_t DdrBuffersAlloc(void** appFbsVirtual, uint32_t*  apFbsPhysical);

/************************************************************************/
/** Frees DDR buffers for one ISP stream.
  * 
  * \param  appFbsVirtual array of virtual buffer pointers to be filled
  * \param  apFbsPhysical array of buffer physical addresses to be filled
  ************************************************************************/
void DdrBuffersFree(void** appFbsVirtual, uint32_t*  apFbsPhysical);

#ifndef __STANDALONE__
/************************************************************************/
/** SIGINT handler.
  * 
  * \param  aSigNo 
  ************************************************************************/
void SigintHandler(int aSigNo);

/************************************************************************/
/** SIGINT handler.
  * 
  * \param  aSigNo 
  * 
  * \return SEQ_LIB_SUCCESS if all ok
  *         SEQ_LIB_FAILURE if failed
  ************************************************************************/
int32_t SigintSetup(void);

//***************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

#endif // #ifndef __STANDALONE__

int main(int, char **)
{
  // DDR buffer pointers
  void*     lppFbsVirtual[DDR_BUFFER_CNT]; ///< virtual buffer pointers
  uint32_t  lpFbsPhysical[DDR_BUFFER_CNT]; ///< physical buffer adresses
  
  #ifndef __STANDALONE__
    OAL_MemoryAllocInit();
  #endif
  
  //*** Init DCU Output ***
#ifdef __STANDALONE__
  io::FrameOutputDCU lDcuOutput(WIDTH, 
                                HEIGHT, 
                                io::IO_DATA_DEPTH_08, 
                                CHNL_CNT);
#else // #ifdef __STANDALONE__
  // setup Ctrl+C handler
  if(SigintSetup() != SEQ_LIB_SUCCESS) 
  {
    VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    return -1;
  }
  
  printf("Press Ctrl+C to terminate the demo.\n");
  
  io::FrameOutputV234Fb lDcuOutput(WIDTH, 
                        HEIGHT, 
                        io::IO_DATA_DEPTH_08, 
                        CHNL_CNT);
#endif // else from #ifdef __STANDALONE__
  
  //*** allocate DDR buffers ***
  if(DdrBuffersAlloc(lppFbsVirtual, lpFbsPhysical) != 0)
  {
    VDB_LOG_ERROR("Failed to allocated DDR buffers.");
    return -1;
  } // if failed to allocate buffers
  
  //*** Init ISP input ***
  io::FrameInputISP lIsp;
  io::IspBufferStream lBufferStream;
  
  lBufferStream.mStreamIdx  = 0; 
  lBufferStream.mCnt        = DDR_BUFFER_CNT;       
  lBufferStream.mBaseShift  = 0; 
  lBufferStream.mLineStride = WIDTH * (uint32_t)io::IO_DATA_CH3;
  lBufferStream.mLineCnt    = 720;	//495;//HEIGHT;  
    
  lBufferStream.mpBuffers   = lpFbsPhysical; 
  
  // init stream 0
  lIsp.DdrBuffersSet(lBufferStream);
  
  //*** Start ISP processing ***
  lIsp.Start(); 
    
  // Frame buffer
  void *lpFrame = NULL;

  uint32_t lFrmCnt = 0;
    
  lIsp.StartCam();

double freq = FSL_GetFrequency();    
int64_t prevTick = 0;

  while((lpFrame = lIsp.GetFrame()))
  {
  	int64_t Tick = FSL_Ticks();  //Calculate FPS
	int fps = 1 / ((Tick - prevTick) / freq);
	prevTick = Tick;


    lpFrame = OAL_MemoryReturnAddress(
                          lpFrame, 
                          ACCESS_PHY + 1); // get virtual address

//SONY_Geometry_t fps2 = mFps;

  //Dcu_LCD_Connection_t fps = DCU_FREQDIV_NORMAL;
char str[200];
sprintf(str,"%d FPS",fps);

	cv::Mat frame = cv::Mat(720, 1280, CV_8UC3, lpFrame);

	cv::Mat textImg = cv::Mat::zeros(75, 150, frame.type());  //Create Black Image Matrix for image (y,x)

	cv::putText(textImg, str, cv::Point(10, 45), 1, 2.0,cv::Scalar(0,0,255),2); //Put FPS (x,y)

	cv::flip(textImg,textImg,-1); // Flip 180

	textImg.copyTo(frame(cv::Rect(1130,0,textImg.cols, textImg.rows))); //(x,y) top left orgin


	//cv::putText(frame, str, cv::Point(100, 100), 1, 1, cv::Scalar(200, 100, 50), 2);
	unsigned char *input = (unsigned char*)(frame.data);



    lDcuOutput.PutFrame(input, false);

    lFrmCnt++;

#ifndef __STANDALONE__  
    if(sStop)
    {
      break; // break if Ctrl+C pressed
    } // if Ctrl+C
#endif //#ifndef __STANDALONE__      
  }

  //*** Stop ISP processing ***
  lIsp.Stop();

#ifdef __STANDALONE__  
  for(;;);  // *** don't return ***
#endif // #ifdef __STANDALONE__    
  
  DdrBuffersFree(lppFbsVirtual, lpFbsPhysical);
//#endif 
} // main()

//***************************************************************************

int32_t DdrBuffersAlloc(void** appFbsVirtual, uint32_t*  apFbsPhysical)
{
  int32_t lRet    = 0;
  
  size_t lMemSize = WIDTH * HEIGHT * (uint32_t)io::IO_DATA_CH3;
  
  // allocate buffers & get physical addresses
  for(uint32_t i = 0; i < DDR_BUFFER_CNT; i++)
  {
    appFbsVirtual[i] = OAL_MemoryAllocFlag(
                          lMemSize, 
                          OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|
                          OAL_MEMORY_FLAG_CONTIGUOUS);
    
    if( appFbsVirtual[i] == NULL)
    {
      lRet = -1;
      break;
    }
    apFbsPhysical[i] = (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(
                          appFbsVirtual[i], 
                          ACCESS_PHY); // get physical address

    memset(appFbsVirtual[i], 0, lMemSize);
  } // for all framebuffers
  
  if(lRet != 0)
  {
    DdrBuffersFree(appFbsVirtual, apFbsPhysical);
  }
  
  return lRet;
} // DdrBuffersAlloc()

//***************************************************************************

void DdrBuffersFree(void** appFbsVirtual, uint32_t*  apFbsPhysical)
{
  for(uint32_t i = 0; i < DDR_BUFFER_CNT; i++)
  {
    if(appFbsVirtual[i] != NULL)
    {
      OAL_MemoryFree(appFbsVirtual[i]);
    } // if buffer allocated
    
    appFbsVirtual[i]   = NULL;
    apFbsPhysical[i]   = 0;
  } // for all framebuffers
} // DdrBuffersFree()

//***************************************************************************

#ifndef __STANDALONE__
void SigintHandler(int aSigNo)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

int32_t SigintSetup()
{
  int32_t lRet = SEQ_LIB_SUCCESS;
  
  // prepare internal signal handler
  struct sigaction lSa;
  memset(&lSa, 0, sizeof(lSa));
  lSa.sa_handler = SigintHandler;
  
  if( sigaction(SIGINT, &lSa, NULL) != 0)
  {
    VDB_LOG_ERROR("Failed to register signal handler.\n");
    lRet = SEQ_LIB_FAILURE;
  } // if signal not registered
  
  return lRet;
} // SigintSetup()

//***************************************************************************
#endif // #ifndef __STANDALONE__
