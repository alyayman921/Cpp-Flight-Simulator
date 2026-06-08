#include "readxslx.h"
#include "derivatives.h"
int main(){

const char filename[]="C5A.xlsx"; // filename itself this is a const char pointer to c5a string 

raw_data raw=readxlsx(filename); 
aircraft_data acd=sorting(raw);
int i=0;
while ( i < 62){
//printf("B%d is %f\n",i,raw.B[i]);
	i++;
}
i=0;
while ( i < 3){
printf("V%d is %f\n",i,acd.V[i]);
	i++;
}
while (false){

}
return 0;
}