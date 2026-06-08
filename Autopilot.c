#include "readxslx.h"
#include "derivatives.h"
int main(){

const char filename[]="meta/C5A.xlsx"; // filename itself this is a const char pointer to c5a string 

raw_data raw=readxlsx(filename); 
aircraft_data acd=sorting(raw);

while (false){

}
return 0;
}