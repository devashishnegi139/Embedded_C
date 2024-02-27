#include <stdio.h>
#include <stdint.h>

struct test{
    // put your variables here
    char var1;
    int var2;
    char var3;
    short var4;
};
// __attribute__((packed));
// add the above line before the terminate of struct to see packed structure

int main(){
    // fill values in t1 accordingly
    struct test t1 = {0xa, 0x139, 0xd, 0x5858};
    char *p = (char *)&t1;

    for(unsigned long long i=0; i<sizeof(struct test); i++){
        printf("%p\t%x\n", p, *p);
        if(i!=0 && (i+1)%4==0)
            printf("-----------------------------------------------\n");
        p++;
    }
    return 0;
}