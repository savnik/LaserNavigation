#define	MAXLINESENSORSIZE	30
typedef struct 	{
	  double gain,off;
	  }linesensor_partype;

int calc_linesensor_param(linesensor_partype param[],double white[],double black[],int N);

void linesensor_correct(double output[],int input[], linesensor_partype par[],int N);

typedef struct{
		double linepos_w,left_edge_w,right_edge_w;
		double linepos_b,left_edge_b,right_edge_b;
                int noline_b,noline_w;
                int count;
		double avg_s[MAXLINESENSORSIZE];	
		double k_filt;
	      }find_line_type;

int find_line(find_line_type *p,double line_sensor_val[], int N);
int find_line1(find_line_type *p,double line_sensor_val[], int N);
 
enum linetype{LINE_MIDDLE_B,LINE_LEFT_B,LINE_RIGHT_B, LINE_MIDDLE_W,LINE_LEFT_W,LINE_RIGHT_W,LINE_MIDDLE_B_CAM,LINE_TEST};
