#include<stdio.h>
#include<assert.h>
#include<rtklib.h>

void android_test(void)
{
    char *file = "../data/android_raw.txt";
    printf("Test input raw\n");
    raw_t raw;
    init_raw(&raw, STRFMT_ANDROID);
    FILE *fp = fopen(file, "r");
    assert(fp != NULL);

    int ret = 0;
    while(ret != -2)
    {
        ret = input_rawf(&raw, STRFMT_ANDROID, fp);
        switch (ret)
        {
        case 1:
            printf("input observation data\n");
            break;
        case 2:
            printf("input ephemeris data\n");
            break;
        case 3:
            printf("input sbas data\n");
            break;
        case 9:
            printf("input ion/utc data\n");
            break;
        case -1:
            printf("input error\n");
            break;
        default:
            break;
        }
        
    }
    fclose(fp);

    printf("%s android_test : OK\n",__FILE__);
}

int main(int argc, char **argv)
{
    android_test();
    return 0;
}