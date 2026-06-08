#include "readxslx.h"
#include "derivatives.h"
#include <iostream>

const char filename[]="meta/C5A.xlsx";

int main(){


raw_data raw=readxlsx(filename); 
aircraft_data c5a=sorting(raw);
int i,j;
std::cout<<c5a.SD<<'\n';
while (false){

}
return 0;
}