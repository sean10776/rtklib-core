#include "rtklib.h"

#define AND_GPS 1
#define AND_SBS 2
#define AND_GLO 3
#define AND_QZS 4
#define AND_CMP 5
#define AND_GAL 6

typedef struct{
    int prn, sys, code, LLI;
    double snr, P, L, D;
}and_mea;

typedef struct
{
    int prn, sys, sat, status;
    int frameId, subfmId;
    uint8_t buff[50];
    int buf_len;
} and_nav;

static int getLen(uint8_t *buff){char len[6];memcpy(len, buff, 6);return atoi(len);}
static uint32_t U4(uint8_t *p) {uint32_t u=0;for(int i=0;i<4;i++){u=u<<8|*(p+i);}return u;}
/* change android gnsstype to rtklib type -----------------------------------*/
static int sat_sys(int gnssid){
    switch (gnssid) {
        case AND_GPS: return SYS_GPS;
        case AND_SBS: return SYS_SBS;
        case AND_GAL: return SYS_GAL;
        case AND_CMP: return SYS_CMP;
        case AND_QZS: return SYS_QZS;
        case AND_GLO: return SYS_GLO;
    }
    return 0;
}
/* split android raw measurements ------------------------------------------*/
static void splitMea(char *m, and_mea *mea){
    char buff[50];
    for(int i=0, idx=0, dataIdx=0; m[i]; i++){
        if(m[i] == ',' || m[i] == '\n'){
            switch (dataIdx)
            {
            case 0: mea->prn=atoi(buff); break;
            case 1: mea->sys=sat_sys(atoi(buff)); break;
            case 2: mea->snr=(uint16_t)(atof(buff) / SNR_UNIT + 0.5); break;
            case 3: mea->P=atof(buff); break;
            case 4: mea->L=atof(buff); break;
            case 5: mea->D=atof(buff); break;
            case 6: mea->code=(uint8_t)atoi(buff); break;
            case 7: mea->LLI=(uint8_t)atoi(buff); break;
            }
            idx = 0; dataIdx++;
            memset(buff, 0, sizeof(buff));
        }else{
            buff[idx++] = m[i];
        }

    }
}
/* decode android raw measurements -----------------------------------------*/
static int decodeMeasurements(raw_t *raw, char *meas){
    int n = 0, cpc = 0;
    char* pch = strtok(meas, "\n");
    and_mea mea;
    do{
        splitMea(pch, &mea);
        int sys, prn, sat, code, idx;
        sys = mea.sys;
        prn = mea.prn + (sys==SYS_QZS ? 192:0);
        if(!(sat=satno(sys, prn))){
            if (sys==SYS_GLO&&prn==255) {
                continue; /* suppress warning for unknown glo satellite */
            }
            trace(2,"sat number error: sys=%2d prn=%2d\n",sys,prn);
            continue;
        }
        code = mea.code;
        if ((idx=code2idx(sys,code))<0) {
            trace(2,"signal error: sat=%2d idx=%d\n",sat,idx);
            continue;
        }
        int i;
        for(i = 0; i < n; i++){  //同衛星不同載波判斷
            if (raw->obs.data[i].sat==sat) break;
        }
        if(i >= n){
            raw->obs.data[n].time=raw->time;
            raw->obs.data[n].sat=sat;
            raw->obs.data[n].rcv=0;
            for(int j = 0; j < NFREQ+NEXOBS;j++){
                raw->obs.data[n].L[j]=raw->obs.data[n].P[j]=0.0;
                raw->obs.data[n].Lstd[j]=raw->obs.data[n].Pstd[j]=0;
                raw->obs.data[n].D[j]=0.0;
                raw->obs.data[n].SNR[j]=raw->obs.data[n].LLI[j]=0;
                raw->obs.data[n].code[j]=CODE_NONE;
            }
            n++;
        }
        raw->obs.data[i].P[idx]=mea.P; 
        raw->obs.data[i].L[idx]=mea.L; if(mea.L != 0)cpc++;
        raw->obs.data[i].D[idx]=mea.D; 
        raw->obs.data[i].SNR[idx]=mea.snr;
        raw->obs.data[i].LLI[idx]=mea.LLI;
        raw->obs.data[i].code[idx]=code;
        if(mea.L != 0.0) raw->lockflag[sat-1][idx]=0;
        trace(3, "decode_and: sys=%d svid=%d idx=%d P = %.3f L = %.3f D = %.3f\n",
              sys, prn, idx, mea.P, mea.L, mea.D);
    } while ((pch = strtok(NULL , "\n")) && n < MAXOBS);
    raw->obs.n = n;
    return 1;
}
/* split android raw measurements ------------------------------------------*/
static int splitNav(char *raw, and_nav *nav){
    char buff[5] = "";
    int dataSize = 0;
    memset(nav->buff, 0, 50);
    for(int i = 0, idx = 0; raw[i]; i++){
        if(raw[i] == ',' || raw[i] == '\r' || raw[i] == '\n'){
            switch (dataSize)
            {
            case 0: nav->prn=atoi(buff); break;
            case 1: nav->sys=sat_sys(atoi(buff)>>8); break;
            case 2: nav->status=(atoi(buff) != 0); break;
            case 3: nav->frameId=atoi(buff); break;
            case 4: nav->subfmId=atoi(buff); break;
            default:nav->buff[dataSize-5] = (uint8_t) atoi(buff);
            }
            idx = 0; dataSize++;
            memset(buff, 0, sizeof(buff));
        }else{
            buff[idx++]=raw[i];
        }
    }
    nav->sat = satno(nav->sys, nav->prn);
    nav->buf_len = dataSize-5;
    return nav->status;
}
/* UTC 8-bit week -> full week -----------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;
    
    time2gpst(time,&week);
    utc[3]+=week/256*256;
    if      (utc[3]<week-127) utc[3]+=256.0;
    else if (utc[3]>week+127) utc[3]-=256.0;
    utc[5]+=utc[3]/256*256;
    if      (utc[5]<utc[3]-127) utc[5]+=256.0;
    else if (utc[5]>utc[3]+127) utc[5]-=256.0;
}
/* decode GPS/QZSS ephemeris -------------------------------------------------*/
static int decode_eph(raw_t *raw, int sat)
{
    eph_t eph={0};
    
    if (!decode_frame(raw->subfrm[sat-1],&eph,NULL,NULL,NULL)) return 0;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&
            eph.iodc==raw->nav.eph[sat-1].iodc&&
            timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1].toc)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode GPS/QZSS ION/UTC parameters ----------------------------------------*/
static int decode_ionutc(raw_t *raw, int sat)
{
    double ion[8],utc[8];
    int sys=satsys(sat,NULL);
    
    if (!decode_frame(raw->subfrm[sat-1],NULL,NULL,ion,utc)) return 0;
    
    adj_utcweek(raw->time,utc);
    if (sys==SYS_QZS) {
        matcpy(raw->nav.ion_qzs,ion,8,1);
        matcpy(raw->nav.utc_qzs,utc,8,1);
    }
    else {
        matcpy(raw->nav.ion_gps,ion,8,1);
        matcpy(raw->nav.utc_gps,utc,8,1);
    }
    return 9;
}
/* decode GPS/QZSS navigation data -------------------------------------------*/
static int decode_nav(raw_t *raw, and_nav *nav){
    uint8_t buff[30] = {0}, *p = nav->buff;
    int i,id,ret,sat = nav->sat;
    if(nav->buf_len != 40){
        trace(-2, "len error");
        return -1;
    }
    for(i=0;i<10;i++,p+=4){
        setbitu(buff,24*i,24,U4(p)>>6);
    }
    id=nav->subfmId;
    memcpy(raw->subfrm[sat-1]+(id-1)*30, buff, 30);
    if(id==3){
        return decode_eph(raw,sat);
    }
    if(id==4||id==5){
        ret=decode_ionutc(raw,sat);
        memset(raw->subfrm[sat-1]+(id-1)*30,0,30);
        return ret;
    }
    return 0;
}
/* decode Galileo I/NAV navigation data --------------------------------------*/
static int decode_enav(raw_t *raw, and_nav *nav){
    return 1;
}
/* decode BDS navigation data ------------------------------------------------*/
static int decode_cnav(raw_t *raw, and_nav *nav){
    eph_t eph={0};
    double ion[8],utc[8];
    uint8_t *p=nav->buff,buff[38]={0};
    int i,id,pgn,prn,sat;
    if(nav->buf_len != 40){
        trace(-2, "len error");
        return -1;
    }
    for (i=0;i<10;i++,p+=4) {
        setbitu(buff,30*i,30,U4(p));
    }
    id=nav->subfmId;
    sat=nav->sat;
    prn=nav->prn;
    if(6 <= prn && prn <= 58){ /* IGSO/MEO */
        memcpy(raw->subfrm[sat-1]+(id-1)*38,buff,38);
        
        if (id==3) {
            if (!decode_bds_d1(raw->subfrm[sat-1],&eph,NULL,NULL)) return 0;
        }
        else if (id==5) {
            if (!decode_bds_d1(raw->subfrm[sat-1],NULL,ion,utc)) return 0;
            matcpy(raw->nav.ion_cmp,ion,8,1);
            matcpy(raw->nav.utc_cmp,utc,8,1);
            return 9;
        }
        else return 0;
    }
    else { /* GEO */
        pgn=getbitu(buff,42,4); /* page numuber */
        
        if (id==1&&1<=pgn&&pgn<=10) {
            memcpy(raw->subfrm[sat-1]+(pgn-1)*38,buff,38);
            if (pgn!=10) return 0;
            if (!decode_bds_d2(raw->subfrm[sat-1],&eph,NULL)) return 0;
        }
        else if (id==5&&pgn==102) {
            memcpy(raw->subfrm[sat-1]+10*38,buff,38);
            if (!decode_bds_d2(raw->subfrm[sat-1],NULL,utc)) return 0;
            matcpy(raw->nav.utc_cmp,utc,8,1);
            return 9;
        }
        else return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode GLONASS navigation data --------------------------------------------*/
static int decode_gnav(raw_t *raw, and_nav *nav){
    geph_t geph={0};
    double utc_glo[8]={0};
    int i,m,prn, sat;
    uint8_t *p=nav->buff, buff[11],*fid;
    prn=nav->prn;
    sat=nav->sat;
    
    if (nav->buf_len<11) {
        trace(2,"gnav length error: len=%d\n",raw->len);
        return -1;
    }
    for (i=0;i<nav->buf_len;i++)
        buff[i]=p[i];

    /* test hamming of GLONASS string */
    if (!test_glostr(buff)) {
        trace(2,"gnav hamming error: sat=%2d\n", sat);
        return -1;
    }

    m=nav->subfmId;
    if (m<1||15<m) {
        trace(2,"gnav string no error: sat=%2d\n", sat);
        return -1;
    }
    /* flush frame buffer if frame-ID changed */
    fid=raw->subfrm[sat-1]+150;
    if (fid[0]!= (uint8_t)nav->frameId) {
        for (i=0;i<4;i++) memset(raw->subfrm[sat-1]+i*10,0,10);
        *fid = nav->frameId;
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    
    if (m==4) {
        /* decode GLONASS ephemeris strings */
        geph.tof=raw->time;
        if (!decode_glostr(raw->subfrm[sat-1],&geph,NULL)||geph.sat!=sat) {
            return 0;
        }
        geph.frq=nav->buff[3]-7;
        
        if (!strstr(raw->opt,"-EPHALL")) {
            if (geph.iode==raw->nav.geph[prn-1].iode) return 0;
        }
        raw->nav.geph[prn-1]=geph;
        raw->ephsat=sat;
        raw->ephset=0;
        return 2;
    }
    else if (m==5) {
        if (!decode_glostr(raw->subfrm[sat-1],NULL,utc_glo)) return 0;
        matcpy(raw->nav.utc_glo,utc_glo,8,1);
        return 9;
    }
    return 0; 
}
/* decode SBAS navigation data -----------------------------------------------*/
static int decode_snav(raw_t *raw, and_nav *nav){
    return 1;    
}
/* decode android navigation message ---------------------------------------*/
static int decodeNavigation(raw_t *raw, char *navs){
    and_nav nav;
    splitNav(navs, &nav);
    // showNav(nav); //delete
    int sys = nav.sys, sat;
    if(!(sat=satno(sys, nav.prn))){
        if (sys==SYS_GLO&&nav.prn==255) {
            return 0; /* suppress warning for unknown glo satellite */
        }
        trace(2,"sat number error: sys=%2d prn=%2d\n",sys, nav.prn);
        return 0;
    }
    
    switch (sys) {
        case SYS_GPS: return decode_nav (raw, &nav);
        case SYS_QZS: return decode_nav (raw, &nav);
        case SYS_GAL: return decode_enav(raw, &nav);
        case SYS_CMP: return decode_cnav(raw, &nav);
        case SYS_GLO: return decode_gnav(raw, &nav);
        case SYS_SBS: return decode_snav(raw, &nav);
    }
    return 0;
}
/* decode android message --------------------------------------------------*/
static int decode_and(raw_t *raw){
    char *p = raw->buff+6, buff[20] = {0};
    if(p[0] == '?'){ //raw measurement
        int i,j, week;double sec;
        for(i = 1, j = 0; p[i] != '#';i++){
            if(p[i] == '@'){
                week = atoi(buff);
                memset(buff, 0, 20);
                j = 0;
                if(week <= 0)return 0;
            } else{
                buff[j++] = p[i];
            }
        }
        sec = atof(buff);
        raw->time = gpst2time(week, sec);
        char *mea = (char*)calloc( raw->len-6-i+1, sizeof(char));
        memcpy(mea, p+i+1, raw->len-6-i);
        // showmsg("input android fulldata:\n%s", mea);
        return decodeMeasurements(raw, mea);
    }else if(p[0] =='/'){ //navigation message
        char *navs = (char*)calloc( raw->len-6+1, sizeof(char));
        memcpy(navs, p+1, raw->len-6-1);
        return decodeNavigation(raw, navs);
    }
    return 0;
}
/* sync head -----------------------------------------------------------------*/
static int sync_and(uint8_t *raw, uint8_t data){
    if(data != '!')return 0;
    raw[0] = data;
    return 1;
}
/* input ublox raw message from stream -----------------------------------------
* fetch next ublox raw data and input a message from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*-----------------------------------------------------------------------------*/
extern int input_and(raw_t *raw, uint8_t data){
    int status = 0, i, j;
    char c = (unsigned char)data;
    if(raw->nbyte == 0) {
        if (sync_and(raw->buff,(uint8_t)data))
            raw->nbyte++;
        return status;
    }
    raw->buff[raw->nbyte++] = data;
    if(raw->nbyte == 7){
        raw->len = getLen(raw->buff+1);
    }
    if(raw->nbyte<7 || raw->nbyte < raw->len)return 0;
    raw->nbyte = 0;
    return decode_and(raw);
}
/*input and raw message from file -------------------------------------------
* fetch next ublox raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_andf(raw_t *raw, FILE *fp){
    int i,data;
    
    trace(4,"input_andf:\n");
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_and(raw->buff,(uint8_t)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+1,1,6,fp)<6) return -2;
    raw->nbyte=7;
    raw->len = getLen(raw->buff+1);
    i = raw->nbyte;
    if (fread(raw->buff+i,1,raw->len-i,fp)<(size_t)(raw->len-i)) return -2;
    raw->nbyte=0;
    return decode_and(raw);
}