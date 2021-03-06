/**
  ******************************************************************************
  * @file    SIN_Tab.c 
  * @author  A. Andreev
  * @version V1.0.0
  * @date    2012-01-20
  * @brief
  ******************************************************************************
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "SIN_Tab.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// Sine wave table in 1Q15 bit format
const int16_t SinTab[251]={
0	,
206	,
412	,
618	,
823	,
1029	,
1235	,
1441	,
1646	,
1852	,
2058	,
2263	,
2468	,
2674	,
2879	,
3084	,
3289	,
3493	,
3698	,
3903	,
4107	,
4311	,
4515	,
4719	,
4923	,
5126	,
5329	,
5532	,
5735	,
5938	,
6140	,
6342	,
6544	,
6746	,
6947	,
7148	,
7349	,
7549	,
7750	,
7949	,
8149	,
8348	,
8547	,
8746	,
8944	,
9142	,
9340	,
9537	,
9733	,
9930	,
10126	,
10321	,
10517	,
10711	,
10906	,
11100	,
11293	,
11486	,
11679	,
11871	,
12063	,
12254	,
12445	,
12635	,
12825	,
13014	,
13202	,
13391	,
13578	,
13765	,
13952	,
14138	,
14323	,
14508	,
14693	,
14876	,
15060	,
15242	,
15424	,
15605	,
15786	,
15966	,
16146	,
16325	,
16503	,
16680	,
16857	,
17033	,
17209	,
17384	,
17558	,
17731	,
17904	,
18076	,
18248	,
18418	,
18588	,
18757	,
18926	,
19094	,
19261	,
19427	,
19592	,
19757	,
19921	,
20084	,
20246	,
20408	,
20568	,
20728	,
20887	,
21045	,
21203	,
21359	,
21515	,
21670	,
21824	,
21977	,
22129	,
22281	,
22431	,
22581	,
22730	,
22877	,
23024	,
23170	,
23316	,
23460	,
23603	,
23745	,
23887	,
24027	,
24167	,
24305	,
24443	,
24580	,
24715	,
24850	,
24984	,
25116	,
25248	,
25379	,
25509	,
25637	,
25765	,
25892	,
26017	,
26142	,
26266	,
26388	,
26510	,
26630	,
26750	,
26868	,
26986	,
27102	,
27217	,
27331	,
27444	,
27556	,
27667	,
27777	,
27885	,
27993	,
28099	,
28205	,
28309	,
28412	,
28514	,
28615	,
28715	,
28813	,
28911	,
29007	,
29102	,
29197	,
29289	,
29381	,
29472	,
29561	,
29649	,
29736	,
29822	,
29907	,
29991	,
30073	,
30154	,
30234	,
30313	,
30391	,
30467	,
30542	,
30616	,
30689	,
30760	,
30831	,
30900	,
30968	,
31035	,
31100	,
31164	,
31227	,
31289	,
31350	,
31409	,
31467	,
31524	,
31579	,
31634	,
31687	,
31739	,
31789	,
31838	,
31886	,
31933	,
31979	,
32023	,
32066	,
32108	,
32148	,
32188	,
32226	,
32262	,
32298	,
32332	,
32365	,
32396	,
32426	,
32455	,
32483	,
32510	,
32535	,
32559	,
32581	,
32603	,
32623	,
32641	,
32659	,
32675	,
32690	,
32703	,
32716	,
32727	,
32736	,
32745	,
32752	,
32758	,
32762	,
32765	,
32767	,
32767	};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*****************************************************************************
  * @brief
  * @param  None
  * @retval None
*****************************************************************************/
int16_t Sin_Tab_Read(int16_t ptr)
{
 if(ptr < 500){							
	if(ptr <= 250)	return(SinTab[ptr]); 	      //sin in Quadrant 1
	else		return(SinTab[500 - ptr]);    //sin in Quadrant 2
 }
 else{
	if(ptr <= 750)	return(-SinTab[ptr - 500]);   //sin in Quadrant 3
 	else		return(-SinTab[1000 - ptr]);  //sin in Quadrant 4
  }
}
/*****************************************************************************/
/******************************** END OF FILE *********************************/
