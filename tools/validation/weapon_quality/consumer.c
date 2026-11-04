/* Standalone C99 public behavior and provenance from the same process. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <string.h>
#define main m03_functional_main
#include "functional_c.inc"
#undef main
static int json_string(FILE* out,const char* value) {
    const unsigned char* p=(const unsigned char*)value;
    if(fputc('"',out)==EOF)return 0;
    for(;*p;++p) {
        if(*p=='"'||*p=='\\') {if(fputc('\\',out)==EOF)return 0;}
        if(*p<32) {if(fprintf(out,"\\u%04x",*p)<0)return 0;}
        else if(fputc(*p,out)==EOF)return 0;
    }
    return fputc('"',out)!=EOF;
}
int main(int argc,char** argv) {
    const char* names[]={"libfsim.dll","libJSBSim.dll"};
    char data[32768];char* args[]={argv[0],data};FILE* out;int status;unsigned i;
    if(argc!=3)return 2;
    if(snprintf(data,sizeof(data),"%s/share/flightsim/jsbsim",argv[1])<0||strlen(argv[1])+24>=sizeof(data))return 3;
    status=m03_functional_main(2,args);if(status)return status;
    out=fopen(argv[2],"wb");if(!out)return 4;
    fputs("{\"modules\":[",out);
    for(i=0;i<2;++i) {
        wchar_t path[32768];char utf8[131072];DWORD length;HMODULE module=GetModuleHandleA(names[i]);
        if(!module){fclose(out);return 5;}
        length=GetModuleFileNameW(module,path,32768);
        if(!length||length>=32768||!WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,path,-1,utf8,sizeof(utf8),NULL,NULL)) {
            fclose(out);return 6;
        }
        if(i)fputc(',',out);
        fputs("{\"name\":",out);json_string(out,names[i]);fputs(",\"path\":",out);json_string(out,utf8);fputc('}',out);
    }
    fputs("]}\n",out);status=ferror(out);if(fclose(out))status=1;return status?7:0;
}
