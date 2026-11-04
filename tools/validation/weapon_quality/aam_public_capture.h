/* Public-consumer observations only. No simulation equations or private API. */
#ifndef M03_AAM_PUBLIC_CAPTURE_H
#define M03_AAM_PUBLIC_CAPTURE_H
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <wctype.h>

static int aam_safe_path(const wchar_t* value) {
    const wchar_t* start=value;const wchar_t* p=value;
    for(;;++p)if(*p==L'/'||*p==L'\\'||!*p) {
        if(p-start==6) {
            const wchar_t* word=L".codex";unsigned i=0;
            for(;i<6&&towlower(start[i])==(wint_t)word[i];++i){}
            if(i==6)return 0;
        }
        if(!*p)break;
        start=p+1;
    }
    return 1;
}
/* Windows CNG, with streamed bytes. No private SDK dependency. */
static int aam_sha256(const wchar_t* path,char output[65]) {
    BCRYPT_ALG_HANDLE algorithm=NULL;BCRYPT_HASH_HANDLE hash=NULL;
    HANDLE input=INVALID_HANDLE_VALUE;unsigned char buffer[65536],digest[32];DWORD count=0;unsigned i;int ok=0;
    if(!aam_safe_path(path))return 0;
    input=CreateFileW(path,GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);
    if(input==INVALID_HANDLE_VALUE)goto finish;
    if(BCryptOpenAlgorithmProvider(&algorithm,BCRYPT_SHA256_ALGORITHM,NULL,0)!=0)goto finish;
    if(BCryptCreateHash(algorithm,&hash,NULL,0,NULL,0,0)!=0)goto finish;
    for(;;) {
        if(!ReadFile(input,buffer,sizeof(buffer),&count,NULL))goto finish;
        if(!count)break;
        if(BCryptHashData(hash,buffer,count,0)!=0)goto finish;
    }
    if(BCryptFinishHash(hash,digest,sizeof(digest),0)!=0)goto finish;
    for(i=0;i<32;++i)sprintf(output+2*i,"%02x",(unsigned)digest[i]);
    output[64]='\0';ok=1;
finish:
    if(hash)BCryptDestroyHash(hash);
    if(algorithm)BCryptCloseAlgorithmProvider(algorithm,0);
    if(input!=INVALID_HANDLE_VALUE)CloseHandle(input);
    return ok;
}
static int aam_utf8_sha256(const char* path,char output[65]) {
    wchar_t native[32768];
    if(!MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path,-1,native,32768))return 0;
    return aam_sha256(native,output);
}

static int aam_json_string(FILE* file,const char* text) {
    const unsigned char* p=(const unsigned char*)text;
    if(fputc('"',file)==EOF)return 0;
    for(;*p;++p) {
        if(*p=='"'||*p=='\\')if(fputc('\\',file)==EOF)return 0;
        if(*p<32){if(fprintf(file,"\\u%04x",(unsigned)*p)<0)return 0;}
        else if(fputc(*p,file)==EOF)return 0;
    }
    return fputc('"',file)!=EOF;
}
static int aam_public_capture(const char* path,const char* language,const char* card,const char* model,const char* epoch,
    uint32_t weapon,uint32_t target,unsigned checks,unsigned observations,unsigned controls,
    unsigned tracking,unsigned burning,unsigned damage,const char* terminal,double launch,double stop,const char* directory) {
    const wchar_t* names[]={NULL,L"libfsim.dll",L"libJSBSim.dll"};
    FILE* file=fopen(path,"wb");unsigned i;int failed=0;
    if(!file)return 0;
    fputs("{\"schema\":\"fsim.sim-m03.public-aam-capture.v1\",\"language\":",file);aam_json_string(file,language);
    fputs(",\"model\":",file);aam_json_string(file,card);fputs(",\"native_model\":",file);aam_json_string(file,model);
    fputs(",\"input_epoch_sha256\":",file);aam_json_string(file,epoch);
    fprintf(file,",\"weapon\":%u,\"target\":%u,\"checks\":%u,\"observations\":%u,\"control_samples\":%u,\"tracking_samples\":%u,\"burning_samples\":%u,\"damage_events\":%u,\"launch_s\":%.17g,\"stop_s\":%.17g,\"terminal\":",
        weapon,target,checks,observations,controls,tracking,burning,damage,launch,stop);
    aam_json_string(file,terminal);fputs(",\"modules\":[",file);
    for(i=0;i<3;++i) {
        const HMODULE module=names[i]?GetModuleHandleW(names[i]):NULL;
        wchar_t native[32768];char utf8[131072],digest[65];DWORD count;
        if(names[i]&&!module){failed=1;break;}
        count=GetModuleFileNameW(module,native,32768);
        if(!count||count>=32768||!WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,native,-1,utf8,sizeof(utf8),NULL,NULL)) {failed=1;break;}
        if(i)fputc(',',file);
        if(!aam_sha256(native,digest)){failed=1;break;}
        fputs("{\"path\":",file);aam_json_string(file,utf8);fputs(",\"sha256\":",file);aam_json_string(file,digest);fputc('}',file);
    }
    fputs("],\"files\":[",file);
    for(i=0;i<2;++i) {
        const char* name=i?"events.csv":"trace.csv";char full[32768],digest[65];
        const int length=snprintf(full,sizeof(full),"%s/%s",directory,name);
        if(length<0||(size_t)length>=sizeof(full)||!aam_utf8_sha256(full,digest)){failed=1;break;}
        if(i)fputc(',',file);
        fputs("{\"path\":",file);aam_json_string(file,name);
        fputs(",\"sha256\":",file);aam_json_string(file,digest);fputc('}',file);
    }
    fputs("]}\n",file);if(ferror(file))failed=1;if(fclose(file))failed=1;
    return !failed;
}
#endif
