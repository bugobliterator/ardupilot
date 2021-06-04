/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#include "AP_ONVIF.h"
#include <AP_ONVIF/DeviceBinding.nsmap>

#include "onvifhelpers.h"
// For ChibiOS we will use HW RND # generator
#include <stdlib.h> //rand()


#ifndef USERNAME
#define USERNAME "admin"
#endif

#ifndef PASSWORD
#define PASSWORD "admin"
#endif

#ifndef ONVIF_HOSTNAME
#define ONVIF_HOSTNAME "http://10.211.55.3:10000/onvif/device_service"
#endif

#ifndef PRINT
#define PRINT(fmt,args...) do {printf(fmt "\n", ## args); } while(0)
#endif

const char *wsse_PasswordDigestURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest";
const char *wsse_Base64BinaryURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary";

// static AP _ONVIF *_singleton;
extern const AP_HAL::HAL &hal;

bool AP_ONVIF::init()
{
    srand ((time_t)(hal.util->get_hw_rtc()/1000000ULL));
    soap = soap_new1(SOAP_XML_STRICT | SOAP_XML_CANONICAL | SOAP_C_UTFSTRING);
    soap->connect_timeout = soap->recv_timeout = soap->send_timeout = 10; // 10 sec

    proxy_device = new DeviceBindingProxy(soap);
    proxy_media = new MediaBindingProxy(soap);
    proxy_ptz = new PTZBindingProxy(soap);

    if (proxy_device == nullptr ||
        proxy_media == nullptr ||
        proxy_ptz == nullptr) {
        AP_HAL::panic("AP_ONVIF: Failed to allocate gSOAP Proxy objects.");
        return false;
    }
    /// TODO: Need to find a way to store this in parameter system
    //  or it could be just storage, we will see
    proxy_device->soap_endpoint = ONVIF_HOSTNAME;
    if (!probe_onvif_server()) {
        PRINT("Failed to probe onvif server.");
        return false;
    }
    return true;
}

void AP_ONVIF::report_error()
{
    PRINT("ONVIF ERROR:\n");
    if (soap_check_state(soap)) {
        PRINT("Error: soap struct state not initialized");
    } else if (soap->error) {
        const char **c, *v = NULL, *s, *d;
        c = soap_faultcode(soap);
        if (!*c) {
            soap_set_fault(soap);
            c = soap_faultcode(soap);
        }
        if (soap->version == 2) {
            v = soap_fault_subcode(soap);
        }
        s = soap_fault_string(soap);
        d = soap_fault_detail(soap);
        PRINT("%s%d fault %s [%s]\n%s\nDetail: %s",(soap->version ? "SOAP 1." : "Error "),
                                                      (soap->version ? (int)soap->version : soap->error),
                                                      *c, (v ? v : "no subcode"), (s ? s : "[no reason]"),
                                                      (d ? d : "[no detail]"));
    }
}

bool AP_ONVIF::probe_onvif_server()
{
    _tds__GetDeviceInformation GetDeviceInformation;
    _tds__GetDeviceInformationResponse GetDeviceInformationResponse;
    set_credentials();
    if (proxy_device->GetDeviceInformation(&GetDeviceInformation, GetDeviceInformationResponse)) {
        report_error();
        return false;
    }

    PRINT("Manufacturer:    %s",GetDeviceInformationResponse.Manufacturer.c_str());
    PRINT("Model:           %s",GetDeviceInformationResponse.Model.c_str());
    PRINT("FirmwareVersion: %s",GetDeviceInformationResponse.FirmwareVersion.c_str());
    PRINT("SerialNumber:    %s",GetDeviceInformationResponse.SerialNumber.c_str());
    PRINT("HardwareId:      %s",GetDeviceInformationResponse.HardwareId.c_str());

    // get device capabilities and print media
    _tds__GetCapabilities GetCapabilities;
    _tds__GetCapabilitiesResponse GetCapabilitiesResponse;
    set_credentials();
    if (proxy_device->GetCapabilities(&GetCapabilities, GetCapabilitiesResponse)) {
        report_error();
        return false;
    }

    if (!GetCapabilitiesResponse.Capabilities || !GetCapabilitiesResponse.Capabilities->Media) {
        PRINT("Missing device capabilities info");
    } else {
        PRINT("XAddr:        %s", GetCapabilitiesResponse.Capabilities->Media->XAddr.c_str());
        if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities) {
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast) {
                PRINT("RTPMulticast: %s",(*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast ? "yes" : "no"));
            }
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP) {
                PRINT("RTP_TCP:      %s", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP ? "yes" : "no"));
            }
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP) {
                PRINT("RTP_RTSP_TCP: %s", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP ? "yes" : "no"));
            }
        }
    }
    return true;
}

void rand_nonce(char *nonce, size_t noncelen)
{
    size_t i;
    uint32_t r = (uint32_t)(hal.util->get_hw_rtc()/1000000ULL);
    (void)memcpy((void *)nonce, (const void *)&r, 4);
    for (i = 4; i < noncelen; i += 4)
    {
        r = rand();
        (void)memcpy((void *)(nonce + i), (const void *)&r, 4);
    }
}
#define TEST_NONCE "LKqI6G/AikKCQrN0zqZFlg=="
#define TEST_TIME "2010-09-16T07:50:45Z"
#define TEST_PASS "userpassword"
#define TEST_RESULT "tuOSpGlFlIXsozq4HFNeeGeFLEI="
#define TEST 0
void AP_ONVIF::set_credentials()
{
    soap_wsse_delete_Security(soap);

    _wsse__Security *security = soap_wsse_add_Security(soap);
    const char *created = soap_dateTime2s(soap, (time_t)(hal.util->get_hw_rtc()/1000000ULL));
    char HA[SHA1_DIGEST_SIZE] {};
    char HABase64fin[29] {};
    char nonce[16], *nonceBase64;
    sha1_ctx ctx;
    uint16_t HABase64len;
    char *HABase64enc;
    uint16_t noncelen; 

    /* generate a nonce */
    rand_nonce(nonce, 16);

    sha1_begin(&ctx);
#if TEST
    char* test_nonce = (char*)base64_decode((const unsigned char*)TEST_NONCE, strlen(TEST_NONCE), &noncelen);
    sha1_hash((const unsigned char*)test_nonce, noncelen, &ctx);
    sha1_hash((const unsigned char*)TEST_TIME, strlen(TEST_TIME), &ctx);
    sha1_hash((const unsigned char*)TEST_PASS, strlen(TEST_PASS), &ctx);
#else
    sha1_hash((const unsigned char*)nonce, 16, &ctx);
    sha1_hash((const unsigned char*)created, strlen(created), &ctx);
    sha1_hash((const unsigned char*)PASSWORD, strlen(PASSWORD), &ctx);
#endif
    sha1_end((unsigned char*)HA, &ctx);
    nonceBase64 = (char*)base64_encode((unsigned char*)nonce, 16, &noncelen);
    HABase64enc = (char*)base64_encode((unsigned char*)HA, SHA1_DIGEST_SIZE, &HABase64len);
    if (HABase64len > 29) {
        //things have gone truly bad time to panic
        PRINT("Error: Invalid Base64 Encode!");
        free(HABase64enc);
        return;
    }

    memcpy(HABase64fin, HABase64enc, HABase64len);
    PRINT("Created:%s Hash64:%s", created, HABase64fin);

    if (soap_wsse_add_Timestamp(soap, "Time", 10) ||
        soap_wsse_add_UsernameTokenText(soap, "Auth", USERNAME, HABase64fin))
    {
        report_error();
    }
    /* populate the remainder of the password, nonce, and created */
    security->UsernameToken->Password->Type = (char*)wsse_PasswordDigestURI;
    security->UsernameToken->Nonce = (struct wsse__EncodedString*)soap_malloc(soap, sizeof(struct wsse__EncodedString));
    security->UsernameToken->Salt = NULL;
    security->UsernameToken->Iteration = NULL;
    if (!security->UsernameToken->Nonce) {
        return;
    }
    soap_default_wsse__EncodedString(soap, security->UsernameToken->Nonce);
    security->UsernameToken->Nonce->__item = nonceBase64;
    security->UsernameToken->Nonce->EncodingType = (char*)wsse_Base64BinaryURI;
    security->UsernameToken->wsu__Created = soap_strdup(soap, created);
}
