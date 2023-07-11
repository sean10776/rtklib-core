#include<stdio.h>
#include<assert.h>
#include<rtklib.h>

void ntou_api_test(void){
    strinitcom();
    stream_t stream;
    strinit(&stream);
    stropen(&stream, STR_HTTPREQ, STR_MODE_RW, "140.121.130.224:80");

    sol_t sol={0};
    sol.time = timeget();
    sol.stat=1;
    sol.rr[0]=25;
    sol.rr[1]=121;
    sol.rr[2]=0;
    char ntousol[256]={0};
    char buff[1024]={0};
    int n = outntouold(ntousol, &sol, "140.121.130.224", "string123");
    printf("%s\n", ntousol);

    strwrite(&stream, ntousol, n);
    strwrite(&stream, ntousol, n);

    strstatx(&stream, buff);
    printf("%s\n", buff);

    while(strstat(&stream, buff)==1){
        printf("stream reading\n");
        sleepms(1000);
    }
    

    while((n = strread(&stream, buff, 1024))>0){
        printf("read stream %d\n", n);
        char *s = malloc(n+1);
        memcpy(s, buff, n);
        s[n]=0;
        printf("%s\n", s);
    }

    strclose(&stream);
}

void main(int argc, char**argv){
    ntou_api_test();
}