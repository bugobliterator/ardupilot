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


#ifndef ONVIF_HOSTNAME
#define ONVIF_HOSTNAME "http://10.211.55.3:10000/onvif/device_service"
#endif

#ifndef PRINT
#define PRINT do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#endif

bool AP_ONVIF::init()
{
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
        PRINT("Failed to probe onvif server.\n");
        return false;
    }
}

void AP_ONVIF::report_error()
{
    PRINT("ONVIF ERROR:\n");
    if (soap_check_state(soap)) {
        PRINT("Error: soap struct state not initialized\n");
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
        PRINT("%s%d fault %d [%s]\n%s\nDetail: %s\n",(soap->version ? "SOAP 1." : "Error "),
                                                      (soap->version ? (int)soap->version : soap->error),
                                                      *c, (v ? v : "no subcode"), (s ? s : "[no reason]"),
                                                      (d ? d : "[no detail]"));
    }
}

bool AP_ONVIF::probe_onvif_server()
{
    _tds__GetDeviceInformation GetDeviceInformation;
    _tds__GetDeviceInformationResponse GetDeviceInformationResponse;

    if (proxyDevice.GetDeviceInformation(&GetDeviceInformation, GetDeviceInformationResponse)) {
        report_error(soap);
        return false;
    }

    PRINT("Manufacturer:    %s\n",GetDeviceInformationResponse.Manufacturer.c_str());
    PRINT("Model:           %s\n",GetDeviceInformationResponse.Model.c_str());
    PRINT("FirmwareVersion: %s\n",GetDeviceInformationResponse.FirmwareVersion.c_str());
    PRINT("SerialNumber:    %s\n",GetDeviceInformationResponse.SerialNumber.c_str());
    PRINT("HardwareId:      %s\n",GetDeviceInformationResponse.HardwareId.c_str());

    // get device capabilities and print media
    _tds__GetCapabilities GetCapabilities;
    _tds__GetCapabilitiesResponse GetCapabilitiesResponse;

    if (proxyDevice.GetCapabilities(&GetCapabilities, GetCapabilitiesResponse)) {
        report_error(soap);
        return false;
    }

    if (!GetCapabilitiesResponse.Capabilities || !GetCapabilitiesResponse.Capabilities->Media) {
        PRINT("Missing device capabilities info\n");
    } else {
        PRINT("XAddr:        %s\n", GetCapabilitiesResponse.Capabilities->Media->XAddr.c_str());
        if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities) {
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast) {
                PRINT("RTPMulticast: %s\n",(*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast ? "yes" : "no"));
            }
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP) {
                PRINT("RTP_TCP:      %s\n", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP ? "yes" : "no"));
            }
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP) {
                PRINT("RTP_RTSP_TCP: %s\n", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP ? "yes" : "no"));
            }
        }
    }
}

