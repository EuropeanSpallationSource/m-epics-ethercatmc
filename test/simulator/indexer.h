#ifndef INDEXER_H
#define INDEXER_H

int indexerHandleADS_ADR_getUInt(unsigned adsport,
                                 unsigned indexOffset,
                                 unsigned len_in_PLC,
                                 unsigned *uValue);
int indexerHandleADS_ADR_putUInt(unsigned adsport,
                                 unsigned indexOffset,
                                 unsigned len_in_PLC,
                                 unsigned uValue);
int indexerHandleADS_ADR_getFloat(unsigned adsport,
                                  unsigned indexOffset,
                                  unsigned len_in_PLC,
                                  double *fValue);
int indexerHandleADS_ADR_putFloat(unsigned adsport,
                                  unsigned indexOffset,
                                  unsigned len_in_PLC,
                                  double fValue);
int indexerHandleADS_ADR_getString(unsigned adsport,
                                   unsigned indexOffset,
                                   unsigned len_in_PLC,
                                   char **sValue);
int indexerHandleADS_ADR_getMemory(unsigned adsport,
                                   unsigned indexOffset,
                                   unsigned len_in_PLC,
                                   void     *buf);
int indexerHandleADS_ADR_setMemory(unsigned adsport,
                                   unsigned indexOffset,
                                   unsigned len_in_PLC,
                                   void     *buf);

void indexerHandlePLCcycle(void);
#endif
