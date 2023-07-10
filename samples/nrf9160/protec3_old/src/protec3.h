#ifndef GUARD_PROTEC3_H
#define GUARD_PROTEC3_H

#define TS_SEND_GPS (1 << 0)


/* aws_handler: type = 4, topic type = 0
, topic = $aws/things/thingy91/shadow/update/delta, data = {"version":30,"timestamp":1592310876,"state":{"welcome":"aws-iot","TargetState":"77"},"metadata":{"welcome":{"timestamp":1592236132},"TargetState":{"timestamp":1592310876}}}*/

#define JSON_STR "{\"version\":30,\"timestamp\":1592310876,\"state\":{\"welcome\":\"aws-iot\",\"TargetState\":\"77\"},\"metadata\":{\"welcome\":{\"timestamp\":1592236132},\"TargetState\":{\"timestamp\":1592310876}}}"

typedef struct {
        char* TargetState;
        int Pairing;
        char *auth;
        char *data;
        char *iv;
} tState;

typedef struct {
        tState state;
} tDelta;

typedef struct {
        char* user;
        char* device;
        char* time;
} tPairRequest;

void get_cognito_token(void);
int init_device(const char *ID, const char *SIM);

int tls_setup(int fd);

#endif /* GUARD_PROTEC3_H */