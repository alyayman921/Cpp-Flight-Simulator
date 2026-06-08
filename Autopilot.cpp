#include "readxslx.h"
#include "derivatives.h"
#include "rk4.h"
#include <iostream>

const char filename[]="meta/C5A.xlsx";

int main(){


raw_data raw=readxlsx(filename); 
aircraft_data c5a=sorting(raw);
int i,j;
//std::cout<<c5a.SD<<'\n';
//std::cout<<c5a.CD<<'\n';
while (false){

}
return 0;
}