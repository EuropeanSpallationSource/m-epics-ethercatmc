#ifndef CMD_EAT_H
#define CMD_EAT_H

int cmd_EAT(int argc, const char *argv[]);
int motorHandleADS_ADR_getInt(unsigned adsport, unsigned indexGroup,
                              unsigned indexOffset, int *iValue);

int motorHandleADS_ADR_putInt(unsigned adsport, unsigned indexGroup,
                              unsigned indexOffset, int iValue);

int motorHandleADS_ADR_getFloat(unsigned adsport, unsigned indexGroup,
                                unsigned indexOffset, double *fValue);

int motorHandleADS_ADR_putFloat(unsigned adsport, unsigned indexGroup,
                                unsigned indexOffset, double fValue);

#endif
