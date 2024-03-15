#include "rtklib.h"

#define SQR(x)   ((x)*(x))
#define NX 4
#define MAXITR 5
#define HFMAX 600 //hatch filter max count
/* write solution to output stream -------------------------------------------*/
static void writesol(dgpssvr_t *svr)
{
    int i,n,sat,prn;
    char sys, *p=svr->sbuff, ts[32];
    dgps_t data;
    
    svr->sbuff=memset(svr->sbuff, 0, sizeof(svr->sbuff));
    trace(4, "writesol:\n");

    time2str(svr->time, ts, 2);
    p+=sprintf(p,">%s\n",ts);
    for(i=0;i<MAXSAT;i++){
        if(svr->dgps[i].iod){
            data = svr->dgps[i];
            if(svr->opt.hfilter < data.hfc)continue;
            sat=satsys(i+1, &prn);
            sys='N';
            if(sat==SYS_GPS)sys='G';
            if(sat==SYS_GLO)sys='R';
            if(sat==SYS_GAL)sys='E';
            if(sat==SYS_CMP)sys='B';
            time2str(data.t0, ts, 2);
            p+=sprintf(p,"%c%03d %6.3f %s\n",sys,prn,data.prc,ts);
        }
    }
    n=strlen(svr->sbuff);

    strwrite(svr->stream+2, svr->sbuff, n);
}
/* update glonass frequency channel number in raw data struct ----------------*/
static void update_glofcn(dgpssvr_t *svr)
{
    int i,j,sat,frq;
    
    for (i=0;i<MAXPRNGLO;i++) {
        sat=satno(SYS_GLO,i+1);
        
        for (j=0,frq=-999;j<2;j++) {
            if (svr->raw[j].nav.geph[i].sat!=sat) continue;
            frq=svr->raw[j].nav.geph[i].frq;
        }
        if (frq<-7||frq>6) continue;
        
        for (j=0;j<2;j++) {
            if (svr->raw[j].nav.geph[i].sat==sat) continue;
            svr->raw[j].nav.geph[i].sat=sat;
            svr->raw[j].nav.geph[i].frq=frq;
        }
    }
}
/* update observation data ---------------------------------------------------*/
static void update_obs(dgpssvr_t *svr, obs_t *obs, int index, int iobs)
{
    int i,n=0;
    
    if (iobs<MAXOBSBUF) {
        for (i=0;i<obs->n;i++) {
            svr->obs[index][iobs].data[n]=obs->data[i];
            svr->obs[index][iobs].data[n++].rcv=index+1;
        }
        svr->obs[index][iobs].n=n;
        sortobs(&svr->obs[index][iobs]);
    }
    svr->nmsg[index][0]++;
}
/* update ephemeris ----------------------------------------------------------*/
static void update_eph(dgpssvr_t *svr, nav_t *nav, int ephsat, int ephset,
                       int index)
{
    eph_t *eph1,*eph2,*eph3;
    geph_t *geph1,*geph2,*geph3;
    int prn;
    
    if (satsys(ephsat,&prn)!=SYS_GLO) {
        /* svr->nav.eph={current_set1,current_set2,prev_set1,prev_set2} */
        eph1=nav->eph+ephsat-1+MAXSAT*ephset;         /* received */
        eph2=svr->nav.eph+ephsat-1+MAXSAT*ephset;     /* current */
        eph3=svr->nav.eph+ephsat-1+MAXSAT*(2+ephset); /* previous */
        if (eph2->ttr.time==0||
                (eph1->iode!=eph3->iode&&eph1->iode!=eph2->iode)||
                (timediff(eph1->toe,eph3->toe)!=0.0&&
                timediff(eph1->toe,eph2->toe)!=0.0)||
            (timediff(eph1->toc,eph3->toc)!=0.0&&
                timediff(eph1->toc,eph2->toc)!=0.0)) {
            *eph3=*eph2; /* current ->previous */
            *eph2=*eph1; /* received->current */
        }
        svr->nmsg[index][1]++;
    }
    else {
        geph1=nav->geph+prn-1;
        geph2=svr->nav.geph+prn-1;
        geph3=svr->nav.geph+prn-1+MAXPRNGLO;
        if (geph2->tof.time==0||
            (geph1->iode!=geph3->iode&&geph1->iode!=geph2->iode)) {
            *geph3=*geph2;
            *geph2=*geph1;
            update_glofcn(svr);
        }
        svr->nmsg[index][1]++;
    }
}
/* update ion/utc parameters -------------------------------------------------*/
static void update_ionutc(dgpssvr_t *svr, nav_t *nav, int index)
{
    
    matcpy(svr->nav.utc_gps,nav->utc_gps,8,1);
    matcpy(svr->nav.utc_glo,nav->utc_glo,8,1);
    matcpy(svr->nav.utc_gal,nav->utc_gal,8,1);
    matcpy(svr->nav.utc_qzs,nav->utc_qzs,8,1);
    matcpy(svr->nav.utc_cmp,nav->utc_cmp,8,1);
    matcpy(svr->nav.utc_irn,nav->utc_irn,9,1);
    matcpy(svr->nav.utc_sbs,nav->utc_sbs,4,1);
    matcpy(svr->nav.ion_gps,nav->ion_gps,8,1);
    matcpy(svr->nav.ion_gal,nav->ion_gal,4,1);
    matcpy(svr->nav.ion_qzs,nav->ion_qzs,8,1);
    matcpy(svr->nav.ion_cmp,nav->ion_cmp,8,1);
    matcpy(svr->nav.ion_irn,nav->ion_irn,8,1);
    svr->nmsg[index][2]++;
}
/* update antenna position ---------------------------------------------------*/
static void update_antpos(dgpssvr_t *svr, int index)
{
    sta_t *sta;
    double pos[3],del[3]={0},dr[3];
    int i;

    if (svr->format[0]==STRFMT_RTCM2||svr->format[0]==STRFMT_RTCM3) {
        sta=&svr->rtcm[0].sta;
    }
    else {
        sta=&svr->raw[0].sta;
    }
    /* update base station position */
    for (i=0;i<3;i++) {
        svr->rb[i]=sta->pos[i];
    }
    /* antenna delta */
    ecef2pos(svr->rb,pos);
    if (sta->deltype) { /* xyz */
        del[2]=sta->hgt;
        enu2ecef(pos,del,dr);
        for (i=0;i<3;i++) {
            svr->rb[i]+=sta->del[i]+dr[i];
        }
    }
    else { /* enu */
        enu2ecef(pos,sta->del,dr);
        for (i=0;i<3;i++) {
            svr->rb[i]+=dr[i];
        }
    }
    svr->nmsg[index][3]++;
}
/* update rtk server struct --------------------------------------------------*/
static void update_svr(dgpssvr_t *svr, int ret, obs_t *obs, nav_t *nav,
                       int ephsat, int ephset, int index, int iobs)
{
    tracet(4,"updatesvr: ret=%d ephsat=%d ephset=%d index=%d\n",ret,ephsat,
           ephset,index);
    
    if (ret==1) { /* observation data */
        update_obs(svr,obs,index,iobs);
        if(index==0) svr->time = gpst2utc(obs[0].data[0].time);
    }
    else if (ret==2) { /* ephemeris */
        update_eph(svr,nav,ephsat,ephset,index);
    }
    else if (ret==9) { /* ion/utc parameters */
        update_ionutc(svr,nav,index);
    }
    else if (ret==5) { /* antenna postion */
        update_antpos(svr,index);
    }
    else if (ret==-1) { /* error */
        svr->nmsg[index][4]++;
    }
}
/* decode receiver raw/rtcm data ---------------------------------------------*/
static int decoderaw(dgpssvr_t *svr, int index)
{
    obs_t *obs;
    nav_t *nav;
    int i,ret,ephsat,ephset,fobs=0;

    tracet(4, "decoderaw: index=%d\n",index);
    dgpssvrlock(svr);

    for (i=0;i<svr->nb[index];i++){
        /* input rtcm/receiver raw data from stream */
        if (svr->format[index]==STRFMT_RTCM2) {
            ret=input_rtcm2(svr->rtcm+index,svr->buff[index][i]);
            obs=&svr->rtcm[index].obs;
            nav=&svr->rtcm[index].nav;
            ephsat=svr->rtcm[index].ephsat;
            ephset=svr->rtcm[index].ephset;
        }
        else if (svr->format[index]==STRFMT_RTCM3) {
            ret=input_rtcm3(svr->rtcm+index,svr->buff[index][i]);
            obs=&svr->rtcm[index].obs;
            nav=&svr->rtcm[index].nav;
            ephsat=svr->rtcm[index].ephsat;
            ephset=svr->rtcm[index].ephset;
        }
        else {
            ret=input_raw(svr->raw+index,svr->format[index],svr->buff[index][i]);
            obs=&svr->raw[index].obs;
            nav=&svr->raw[index].nav;
            ephsat=svr->raw[index].ephsat;
            ephset=svr->raw[index].ephset;
        }

        /* update rtk server */
        if (ret>0) {
            update_svr(svr,ret,obs,nav,ephsat,ephset,index,fobs);
        }
        /* observation data received */
        if (ret==1) {
            if (fobs<MAXOBSBUF) fobs++; else svr->prcout++;
        }
    }
    svr->nb[index]=0;
    dgpssvrunlock(svr);
    return fobs;
}
/* get group delay parameter (m) ---------------------------------------------*/
static double gettgd(int sat, const nav_t *nav, int type)
{
    int i,sys=satsys(sat,NULL);
    
    if (sys==SYS_GLO) {
        for (i=0;i<nav->ng;i++) {
            if (nav->geph[i].sat==sat) break;
        }
        return (i>=nav->ng)?0.0:-nav->geph[i].dtaun*CLIGHT;
    }
    else {
        for (i=0;i<nav->n;i++) {
            if (nav->eph[i].sat==sat) break;
        }
        return (i>=nav->n)?0.0:nav->eph[i].tgd[type]*CLIGHT;
    }
}
/* iono-free or "pseudo iono-free" pseudorange with code bias correction -----*/
static double prange(const obsd_t *obs, const nav_t *nav, int opt)
{
    double P1,P2,gamma,b1,b2;
    int sat,sys,f2;
    
    sat=obs->sat;
    sys=satsys(sat,NULL);
    P1=obs->P[0];
    f2=seliflc(2,satsys(obs->sat,NULL));
    P2=obs->P[f2];
    
    if (P1==0.0||(opt==IONOOPT_IFLC&&P2==0.0)) return 0.0;
    
    /* P1-C1,P2-C2 DCB correction */
    if (sys==SYS_GPS||sys==SYS_GLO) {
        if (obs->code[0]==CODE_L1C) P1+=nav->cbias[sat-1][1][1]; /* C1->P1 */
        if (obs->code[1]==CODE_L2C) P2+=nav->cbias[sat-1][1][2]; /* C2->P2 */
    }
    if (opt==IONOOPT_IFLC) { /* dual-frequency */
        
        if (sys==SYS_GPS||sys==SYS_QZS) { /* L1-L2 or L1-L5 */
            gamma=f2==1?SQR(FREQL1/FREQL2):SQR(FREQL1/FREQL5);
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_GLO) { /* G1-G2 or G1-G3 */
            gamma=f2==1?SQR(FREQ1_GLO/FREQ2_GLO):SQR(FREQ1_GLO/FREQ2_GLO);
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_GAL) { /* E1-E5b, E1-E5a */
            gamma=f2==1?SQR(FREQL1/FREQE5b):SQR(FREQL1/FREQL5);
            if (f2==1&&getseleph(SYS_GAL)) { /* F/NAV */
                P2-=gettgd(sat,nav,0)-gettgd(sat,nav,1); /* BGD_E5aE5b */
            }
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_CMP) { /* B1-B2 */
            gamma=SQR(((obs->code[0]==CODE_L2I)?FREQ1_CMP:FREQL1)/FREQ2_CMP);
            if      (obs->code[0]==CODE_L2I) b1=gettgd(sat,nav,0); /* TGD_B1I */
            else if (obs->code[0]==CODE_L1P) b1=gettgd(sat,nav,2); /* TGD_B1Cp */
            else b1=gettgd(sat,nav,2)+gettgd(sat,nav,4); /* TGD_B1Cp+ISC_B1Cd */
            b2=gettgd(sat,nav,1); /* TGD_B2I/B2bI (m) */
            return ((P2-gamma*P1)-(b2-gamma*b1))/(1.0-gamma);
        }
        else if (sys==SYS_IRN) { /* L5-S */
            gamma=SQR(FREQL5/FREQs);
            return (P2-gamma*P1)/(1.0-gamma);
        }
    } else { /* single-freq (L1/E1/B1) */
        
        if (sys==SYS_GPS||sys==SYS_QZS) { /* L1 */
            b1=gettgd(sat,nav,0); /* TGD (m) */
            return P1-b1;
        }
        else if (sys==SYS_GLO) { /* G1 */
            gamma=SQR(FREQ1_GLO/FREQ2_GLO);
            b1=gettgd(sat,nav,0); /* -dtaun (m) */
            return P1-b1/(gamma-1.0);
        }
        else if (sys==SYS_GAL) { /* E1 */
            if (getseleph(SYS_GAL)) b1=gettgd(sat,nav,0); /* BGD_E1E5a */
            else                    b1=gettgd(sat,nav,1); /* BGD_E1E5b */
            return P1-b1;
        }
        else if (sys==SYS_CMP) { /* B1I/B1Cp/B1Cd */
            if      (obs->code[0]==CODE_L2I) b1=gettgd(sat,nav,0); /* TGD_B1I */
            else if (obs->code[0]==CODE_L1P) b1=gettgd(sat,nav,2); /* TGD_B1Cp */
            else b1=gettgd(sat,nav,2)+gettgd(sat,nav,4); /* TGD_B1Cp+ISC_B1Cd */
            return P1-b1;
        }
        else if (sys==SYS_IRN) { /* L5 */
            gamma=SQR(FREQs/FREQL5);
            b1=gettgd(sat,nav,0); /* TGD (m) */
            return P1-gamma*b1;
        }
    }
    return P1;
}
/* pseudorange residuals -----------------------------------------------------*/
static int rescode(const obsd_t *obs, int n, const double *rs, 
                   const double *dts, const nav_t *nav, const double *rb,
                   const double *dx, const prcopt_t *opt, double *v, double *H, int *sats){
    gtime_t time;                    
    int i,j,nv,sat;
    double freq,dion,vion,dtr,rr[4],e[3],pos[3],*azel,*r,*Ps;
    azel=mat(2,n); r=mat(1,n); Ps=mat(1,n); 
    trace(3, "rescode: rb=%13.3f %13.3f %13.3f\n",rb[0],rb[1],rb[2]);
    
    for(i=0;i<3;i++)rr[i]=rb[i];//+dx[i];
    dtr=dx[3];

    ecef2pos(rr,pos);
    for(i=nv=0;i<n&&i<2*MAXOBS;i++){
        time=obs[i].time;
        sat=obs[i].sat;
        /* geometric distance and elevation*/
        if((r[nv]=geodist(rs+i*6,rb,e))<=0.0) continue;
        if(satazel(pos,e,azel+i*2)<opt->elmin) continue;

        /* ionospheric correction */
        if (!ionocorr(time,nav,sat,pos,azel+i*2,IONOOPT_BRDC,&dion,&vion)) {
            continue;
        }
        if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;
        dion*=SQR(FREQL1/freq);

        if ((Ps[nv]=prange(obs+i,nav,IONOOPT_IFLC))==0.0)continue;
        
        v[nv]=Ps[nv]-(r[nv]+dtr-CLIGHT*dts[i*2]+dion);
        sats[nv]=i;
        /* design matrix */
        for (j=0;j<NX;j++) {
            H[j+nv*NX]=j<3?-e[j]:(j==3?1.0:0.0);
        }
        nv++;
    }   
    for(i=0;i<nv;i++){
        j=sats[i];
        trace(3,"sat=%3d rs=%13.3f %13.3f %13.3f SB=%13.3f P=%13.3f rd=%13.3f\n",
              obs[j].sat,rs[j*6],rs[1+j*6],rs[2+j*6],r[i],Ps[i],v[i]);
    }
    free(azel); free(r); free(Ps);
    return nv;
}
/* hatch filter caculate -----------------------------------------------------*/
static int hfilter(dgps_t *dgps, double c){
    if(dgps->hfc>0){
        dgps->prc = (dgps->hfc - 1)*dgps->prc + c;
        dgps->prc /= dgps->hfc;
    } else {
        dgps->prc = c;
        dgps->hfc = 1;
    }
    if(dgps->hfc < HFMAX) dgps->hfc++;
}
/* residual dgps caculate ------------------------------------------------------
* compute correction value with pseudorange
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          double *rb       I   reference base position
*          dgps_t *dgps     O   DGNSS correction
*-----------------------------------------------------------------------------*/
static void rdgps(const obsd_t *obs, int n, const nav_t *nav, const prcopt_t *opt, const double *rb, dgps_t *dgps){
    gtime_t time0 = obs->time;
    double *rs,*dts,*var,*H,*v,dx[NX]={0},Q[NX*NX];
    int i,j,info,nv,svh[MAXOBS],*sats=(int *)malloc(n*sizeof(int));

    v=mat(n,1); H=mat(NX,n);
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); //pos/vel //clock bias/drift //varience
    satposs(time0,obs,n,nav,EPHOPT_BRDC,rs,dts,var,svh);

    for(i=0;i<MAXITR;i++){
        nv=rescode(obs,n,rs,dts,nav,rb,dx,opt,v,H,sats);

        if((info=lsq(H,v,NX,nv,dx,Q))){
            trace(2,"lsq error info=%d\n",info);
            return;
        }

        if(abs(dx[3])<1E-4) break;
    }

    int sat;
    for(i=0;i<nv;i++){
        sat = obs[sats[i]].sat;
        dgps[sat-1].t0 = gpst2utc(obs[sats[i]].time);
        dgps[sat-1].iod = 1;
        hfilter(&dgps[sat-1], -v[i]);
    }

    free(rs); free(dts); free(var); free(v); free(H);
}
/* deprecated overtime prc -----------------------------------------------------*/
static void dpprc(dgpssvr_t *svr){
    gtime_t rt, satt;
    rt = svr->time;
    for(int i=0;i<MAXSAT;i++){
        if(!svr->dgps[i].iod)continue; //skip no prc data
        satt = svr->dgps[i].t0;
        if(abs(timediff(rt, satt)) > 20*60){
            svr->dgps[i].iod=0;
            svr->dgps[i].hfc=0;
        }
    }
}
/* send nmea request to base/nrtk input stream -------------------------------*/
static void send_nmea(dgpssvr_t *svr)
{
	sol_t sol_nmea={{0}};

	if (svr->stream[0].state!=1) return;
	sol_nmea.ns=10; /* Some servers don't like when ns = 0 */

	if (svr->nmeareq==1) { /* lat-lon-hgt mode */
		sol_nmea.stat=SOLQ_SINGLE;
		sol_nmea.time=utc2gpst(timeget());
		matcpy(sol_nmea.rr,svr->nmeapos,3,1);
		strsendnmea(svr->stream,&sol_nmea);
	}
}
/* dgps server thread ---------------------------------------------------------*/
#ifdef WIN32
static DWORD WINAPI dgpssvrthread(void *arg)
#else
static void *dgpssvrthread(void *arg)
#endif
{
    dgpssvr_t *svr=(dgpssvr_t*)arg;
    uint8_t *p,*q;
    uint32_t tick,ticknmea,tick1hz;
    int i,j,n,fobs[2]={0},cycle,cputime;
    
    svr->state=1;
    svr->tick=tickget();
    ticknmea=tick1hz=svr->tick-1000;
    
    for (cycle=0;svr->state;cycle++){
        tick=tickget();
        for (i=0;i<2;i++){
            p=svr->buff[i]+svr->nb[i]; q=svr->buff[i]+svr->buffsize;
            /* read receiver raw/rtcm data from input stream */
            if((n=strread(svr->stream+i,p,q-p))<=0){
                continue;
            }
            /* write receiver raw/rtcm data to log stream */
            strwrite(svr->stream+i+3,p,n);
            svr->nb[i]+=n;
        }
        for(i=0;i<2;i++){
            fobs[i]=decoderaw(svr,i);
        }

        if(fobs[0]>0){
            trace(3, "Base pos: %.5f %.5f %.5f\n", svr->rb[0], svr->rb[1], svr->rb[2]);
            if(norm(svr->rb, 3)>0){//check has base pos
                rdgps(svr->obs[0][0].data, svr->obs[0][0].n,&svr->nav, &svr->opt, svr->rb, svr->dgps);
            }else{
                trace(2, "no base position\n");
            }
        }

        dpprc(svr); //deprecated overtime correction
        writesol(svr);
        /* send nmea request to base/nrtk input stream */
        if (svr->nmeacycle>0&&(int)(tick-ticknmea)>=svr->nmeacycle) {
            dgpssvrlock(svr);
            send_nmea(svr);
            dgpssvrunlock(svr);
            ticknmea=tick;
        }
            
        if((cputime=(int)(tickget()-tick))>0) svr->cputime=cputime;
        trace(3, "cputime %d\n",cputime);
        /* sleep until next cycle */
        sleepms(svr->cycle-cputime);
    }
    for (i=0;i<5;i++) strclose(svr->stream+i);
    for (i=0;i<2;i++){
        svr->nb[i]=0;
        free(svr->buff[i]);
        free_raw (svr->raw +i);
        free_rtcm(svr->rtcm+i);
    }
    svr->nsb=0;
    free(svr->sbuff);
    return 1;
}
/* initialize dgps server -------------------------------------------------------
* initialize dgps server
* args   : dgpssvr_t *svr    IO dgps server
* return : status (0:error,1:ok)
*-----------------------------------------------------------------------------*/
extern int dgpssvrinit(dgpssvr_t *svr){
    eph_t  eph0 ={0,-1,-1};
    geph_t geph0={0,-1};
    seph_t seph0={0};
    int i, j;
    tracet(3, "dgpssvrinit:\n");

    svr->state=svr->cycle=0;
    svr->tick=0;
    svr->thread=0;
    svr->cputime=svr->prcout=0;
    for(i=0;i<6;i++)svr->rb[i]=0;
    memset(svr->dgps,0,sizeof(svr->dgps));
    /* proccessing option */
    svr->opt=prcopt_default;
    //buff init
    svr->buffsize=0;
    for(i=0;i<2;i++){
        svr->buff[i]=NULL;
        svr->nb[i]=svr->format[i]=0;
    }
    svr->sbuff=NULL;
    svr->nsb=0;
    for(i=0;i<5;i++)strinit(svr->stream+i);
    //nav init
    memset(&svr->nav,0,sizeof(nav_t));
    if (!(svr->nav.eph =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT*4 ))||
        !(svr->nav.geph=(geph_t *)malloc(sizeof(geph_t)*NSATGLO*2))||
        !(svr->nav.seph=(seph_t *)malloc(sizeof(seph_t)*NSATSBS*2))) {
        tracet(1,"dgpssvrinit: malloc error\n");
        return 0;
    }
    for (i=0;i<MAXSAT*4 ;i++) svr->nav.eph [i]=eph0;
    for (i=0;i<NSATGLO*2;i++) svr->nav.geph[i]=geph0;
    for (i=0;i<NSATSBS*2;i++) svr->nav.seph[i]=seph0;
    svr->nav.n =MAXSAT *2;
    svr->nav.ng=NSATGLO*2;
    svr->nav.ns=NSATSBS*2;
    //obs init
    for (i=0;i<2;i++) for (j=0;j<MAXOBSBUF;j++) {
        if (!(svr->obs[i][j].data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS))) {
            tracet(1,"dgpssvrinit: malloc error\n");
            return 0;
        }
    }
    //raw init
    for (i=0;i<2;i++) {
        memset(svr->raw +i,0,sizeof(raw_t ));
        memset(svr->rtcm+i,0,sizeof(rtcm_t));
    }
    initlock(&svr->lock);
    return 1;
}
/* free dgps server -------------------------------------------------------------
* free dgps server
* args   : dgpssvr_t *svr    IO dgps server
* return : none
*-----------------------------------------------------------------------------*/
extern void dgpssvrfree(dgpssvr_t *svr)
{
    int i,j;
    free(svr->nav.eph );
    free(svr->nav.geph);
    free(svr->nav.seph);
    for (i=0;i<2;i++) for (j=0;j<MAXOBSBUF;j++) {
        free(svr->obs[i][j].data);
    }
}
/* lock/unlock dgps server ------------------------------------------------------
* lock/unlock dgps server
* args   : dgpssvr_t *svr    IO dgps server
* return : status (1:ok 0:error)
*-----------------------------------------------------------------------------*/
extern void dgpssvrlock  (dgpssvr_t *svr) {lock  (&svr->lock);}
extern void dgpssvrunlock(dgpssvr_t *svr) {unlock(&svr->lock);}

/* start dgps server ------------------------------------------------------------
* start dgps server thread
* args   : dgpssvr_t *svr    IO dgps server
*          int     cycle    I  server cycle (ms)
*          int     buffsize I  input buffer size (bytes)
*          int     *strs    I  stream types (STR_???)
*                              types[0]=input stream base station
*                              types[1]=input stream correction
*                              types[2]=output stream solution 1
*                              types[3]=log stream base station
*                              types[4]=log stream correction
*          char    *paths   I  input stream paths
*          int     *format  I  input stream formats (STRFMT_???)
*                              format[0]=input stream base station
*                              format[1]=input stream correction
*          int     nmeacycle I nmea request cycle (ms) (0:no request)
*          int     nmeareq  I  nmea request type
*                              (0:no,1:base pos)
*          double *nmeapos  I  transmitted nmea position (ecef) (m)
*          prcopt_t *prcopt I  rtk processing options
*          char   *errmsg   O  error message
* return : status (1:ok 0:error)
*-----------------------------------------------------------------------------*/
extern int dgpssvrstart(dgpssvr_t *svr, int cycle, int buffsize, int *strs,
                        char **paths, int *formats, int nmeacycle,
                        int nmeareq, const double *nmeapos, prcopt_t *prcopt, char *errmsg)
{
    gtime_t time,time0={0};
    int i,j,rw;
    tracet(3, "dgpssvrstart: cycle%d buffersize=%d\n", cycle, buffsize);
    
    if(svr->state){
        sprintf(errmsg,"server already started");
        return 0;
    }
    strinitcom();
    svr->prcout=0;
    svr->cycle=cycle>1?cycle:1;
    svr->nmeacycle=nmeacycle>1000?nmeacycle:1000;
    svr->nmeareq=nmeareq;
    for (i=0;i<3;i++) svr->nmeapos[i]=nmeapos[i];
    svr->buffsize=buffsize>4096?buffsize:4096;
    for (i=0;i<2;i++) svr->format[i]=formats[i];
    for (i=0;i<2;i++){/* input init */
        if(!(svr->buff[i]=(uint8_t*)malloc(buffsize))){
            tracet(1,"dgpssvrstart: malloc error\n");
            sprintf(errmsg,"dgps server malloc error");
            return 0;
        }
        for (j=0;j<MAXOBS;j++)svr->obs[i][j].n=0;
        for (j=0;j<10;j++)svr->nmsg[i][j]=0;
        //init raw and rtcm
        init_raw(svr->raw+i,formats[i]);
        init_rtcm(svr->rtcm+i);
    }
    /* output init */
    if(!(svr->sbuff=(uint8_t*)malloc(buffsize))){
        tracet(1,"dgpssvrstart: malloc error\n");
        sprintf(errmsg,"dgps server malloc error");
        return 0;
    }
    svr->nsb=0;
    memset(svr->dgps,0,sizeof(svr->dgps));
    /* proccessing option */
    svr->opt=*prcopt;
    /* update navigation data */
    for (i=0;i<MAXSAT*4 ;i++) svr->nav.eph [i].ttr=time0;
    for (i=0;i<NSATGLO*2;i++) svr->nav.geph[i].tof=time0;
    for (i=0;i<NSATSBS*2;i++) svr->nav.seph[i].tof=time0;
    /* reset base pos/vel */
    for(i=0;i<6;i++)svr->rb[i]=0;
    /* open streams */
    for (i=0;i<5;i++){
        rw=i<2?STR_MODE_R:STR_MODE_W;
        if(strs[i]!=STR_FILE)rw|=STR_MODE_W;
        if(!stropen(svr->stream+i,strs[i],rw,paths[i])){
            sprintf(errmsg,"str%d open error path=%s",i+1,paths[i]);
            for (i--;i>=0;i--) strclose(svr->stream+i);
            return 0;
        }
        /* set initial time for rtcm and raw */
        if (i<2) {
            time=utc2gpst(timeget());
            svr->raw [i].time=strs[i]==STR_FILE?strgettime(svr->stream+i):time;
            svr->rtcm[i].time=strs[i]==STR_FILE?strgettime(svr->stream+i):time;
        }
    }
    /* sync input streams */
    strsync(svr->stream,svr->stream+1);
    /* create dgps server thread */
#ifdef WIN32
    if (!(svr->thread=CreateThread(NULL,0,dgpssvrthread,svr,0,NULL))) {
#else
    if (pthread_create(&svr->thread,NULL,dgpssvrthread,svr)) {
#endif
        for (i=0;i<MAXSTRRTK;i++) strclose(svr->stream+i);
        sprintf(errmsg,"thread create error\n");
        return 0;
    }
    return 1;
}
/* stop dgps server -------------------------------------------------------------
* start dgps server thread
* args   : dgpssvr_t *svr    IO dgps server
*          char    **cmds   I  input stream stop commands
*                              cmds[1]=input stream base  (NULL: no command)
*                              cmds[2]=input stream ephem (NULL: no command)
* return : none
*-----------------------------------------------------------------------------*/
extern void dgpssvrstop(dgpssvr_t *svr, char **cmds)
{
    int i;
    
    tracet(3,"dgpssvrstop:\n");
    
    /* write stop commands to input streams */
    dgpssvrlock(svr);
    for (i=0;i<2;i++) {
        if (cmds[i]) strsendcmd(svr->stream+i,cmds[i]);
    }
    dgpssvrunlock(svr);
    
    /* stop dgps server */
    svr->state=0;
    
    /* free dgps server thread */
#ifdef WIN32
    WaitForSingleObject(svr->thread,10000);
    CloseHandle(svr->thread);
#else
    pthread_join(svr->thread,NULL);
#endif
}