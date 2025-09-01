#ifndef HTTPS_REQUEST_H
#define HTTPS_REQUEST_H

#ifdef __cplusplus
extern "C" {
#endif

void request_writeBackUART(const char *path, const uint16_t expectedLen);

#ifdef __cplusplus
}
#endif

#endif // HTTPS_REQUEST_H