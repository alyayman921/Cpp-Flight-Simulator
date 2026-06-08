#include "readxslx.h"
#include "derivatives.h"
#include <iostream>

const char filename[]="meta/C5A.xlsx";

int main(){


raw_data raw=readxlsx(filename); 
aircraft_data c5a=sorting(raw);
int i=0;
while (i<3){
std::cout<<c5a.mg0[i]<<'\n';
i++;
}
while (true){
	
}
return 0;
}