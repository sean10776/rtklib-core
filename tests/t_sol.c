#include<stdio.h>
#include<assert.h>
#include<rtklib.h>

void ntou_api_test(void){
    strinitcom();
    stream_t stream;
    strinit(&stream);
    stropen(&stream, STR_HTTPREQ, STR_MODE_RW, "140.121.130.224:80");

    sol_t sol={0};
    sol.stat=1;
    char ntousol[256]={0};
    int n = outntouold(ntousol, &sol, "140.121.130.224", "string");
    printf("%s\n", ntousol);

    strwrite(&stream, ntousol, n);
    strwrite(&stream, ntousol, n);

    strclose(&stream);
}

void main(int argc, char**argv){
    ntou_api_test();
}