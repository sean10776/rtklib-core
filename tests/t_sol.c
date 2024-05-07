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
    char ntousol[256]={0};
    char buff[1024]={0};
    int n = outntouold(ntousol, &sol, "140.121.130.224", "string123");
    printf("%s\n", ntousol);

    strwrite(&stream, ntousol, n);
    strwrite(&stream, ntousol, n);

    strstatx(&stream, buff);
    printf("%s\n", buff);

    if(strstat(&stream, buff)==1){
        printf("stream reading\n");
        sleepms(1000);
    }else{
        printf("stream not reading %s\n", buff);
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
    int sys, prn;
    char id[5]={0};
    printf("%d\n", satid2no("C23"));
    printf("%d\n", satid2no("C37"));
    // int prns[] = {101,118,122,123,132,134,135};
    int prns[] = {63,69,71,78,92};
    for(int i=0;i<sizeof(prns)/sizeof(prns[0]);i++){
        sys = satsys(prns[i], &prn);
        satno2id(prns[i], id);
        printf("%d => %s\n", prns[i], id);
    }
    // ntou_api_test();
}