
#include <AP_HAL/AP_HAL.h>
#include <AP_ONVIF/AP_ONVIF.h> 
#include <AP_ONVIF/devicemgmtDeviceBindingProxy.h>
#include <AP_ONVIF/DeviceBinding.nsmap>
void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


#ifndef ONVIF_HOSTNAME
#define ONVIF_HOSTNAME "10.211.55.3:10000"
#endif

// to report an error
static void report_error(struct soap *soap)
{
  std::cerr << "Oops, something went wrong:" << std::endl;
  soap_stream_fault(soap, std::cerr);
  exit(EXIT_FAILURE);
}

void setup()
{
    hal.console->printf("AP_ONVIF library test\n");
}

void loop()
{
    // create a context with strict XML validation and exclusive XML canonicalization for WS-Security enabled
    struct soap *soap = soap_new1(SOAP_XML_STRICT | SOAP_XML_CANONICAL | SOAP_C_UTFSTRING);
    soap->connect_timeout = soap->recv_timeout = soap->send_timeout = 10; // 10 sec
    // soap_register_plugin(soap, soap_wsse);

    // create the proxies to access the ONVIF service API at HOSTNAME
    DeviceBindingProxy proxyDevice(soap);
    // MediaBindingProxy proxyMedia(soap);

    // get device info and print
    proxyDevice.soap_endpoint = ONVIF_HOSTNAME;
    _tds__GetDeviceInformation GetDeviceInformation;
    _tds__GetDeviceInformationResponse GetDeviceInformationResponse;

    if (proxyDevice.GetDeviceInformation(&GetDeviceInformation, GetDeviceInformationResponse))
        report_error(soap);
    // check_response(soap);
    hal.console->printf("Manufacturer:    %s\n",GetDeviceInformationResponse.Manufacturer.c_str());
    hal.console->printf("Model:           %s\n",GetDeviceInformationResponse.Model.c_str());
    hal.console->printf("FirmwareVersion: %s\n",GetDeviceInformationResponse.FirmwareVersion.c_str());
    hal.console->printf("SerialNumber:    %s\n",GetDeviceInformationResponse.SerialNumber.c_str());
    hal.console->printf("HardwareId:      %s\n",GetDeviceInformationResponse.HardwareId.c_str());

    // get device capabilities and print media
    _tds__GetCapabilities GetCapabilities;
    _tds__GetCapabilitiesResponse GetCapabilitiesResponse;

    if (proxyDevice.GetCapabilities(&GetCapabilities, GetCapabilitiesResponse))
        report_error(soap);
    // check_response(soap);
    if (!GetCapabilitiesResponse.Capabilities || !GetCapabilitiesResponse.Capabilities->Media)
    {
        hal.console->printf("Missing device capabilities info\n");
    } else {
        hal.console->printf("XAddr:        %s\n", GetCapabilitiesResponse.Capabilities->Media->XAddr.c_str());
        if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities)
        {
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast)
                hal.console->printf("RTPMulticast: %s\n",(*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast ? "yes" : "no"));
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP)
                hal.console->printf("RTP_TCP:      %s\n", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP ? "yes" : "no"));
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP)
                hal.console->printf("RTP_RTSP_TCP: %s\n", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP ? "yes" : "no"));
        }

        // free all deserialized and managed data, we can still reuse the context and proxies after this
        soap_destroy(soap);
        soap_end(soap);

        // free the shared context, proxy classes must terminate as well after this
        soap_free(soap);
    }

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
