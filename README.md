# WiTTS: Wireless Temperature Tracking System
Please visit the [WiTTS Mobile Application] (https://github.com/esseun/witts_ionic) before continuing further

## Hardware Requirements
- [Texas Instruments TIDA-00217] (http://www.ti.com/tool/TIDA-00217)
- [Texas Instruments MSP430FR5969 LanuchPad Development Kit] (http://www.ti.com/tool/msp-exp430fr5969)
- [1x6 Male-to-Male right-angle Sullins Connector] (http://www.digikey.com/product-detail/en/GRPB061VWCN-RC/S9016E-06-ND/1786469)
- NFC-enabled Android Mobile Device (e.g. Nexus 7 2013)

## Getting Started
- `git clone https://github.com/esseun/witts_msp430fr5969.git witts_msp430fr5969`
- Install the latest version of [Teaxas Instruments Code Composer Studio] (http://www.ti.com/tool/ccstudio)
- Import the witts_msp430fr5969 project using the Code Composer import (existing project) wizard
- Right click the project root directory, then select 'Build Project'
- Connect the TIDA-00217 to the LaunchPad's J21 headers using the Sullins connector
- In CCS, click Run, then Debug
  - The debugger will break on 'main'
  - Click 'Resume' to finish flashing the device
- Test the device with an NFC reader such as [NFC Tools] (https://play.google.com/store/apps/details?id=com.wakdev.wdnfc&hl=en)
  - The device should outputa string in the folowing format: `<9-digit_patient_id> <temperature_in_fahrenheit>`
  - The patient ID is hardcoded and can be changed in the CCS project

## Support
The WiTTS project will not be actively maintained but is open for support. The author
can be reached by email at the address below for user account and/or hardware requests:
- Jason Choi: <esseun@gmail.com>

## Copyright
Copyright 2016© Jason Choi
- Republication or redistribution of WiTTS software content is strictly prohibited without the prior written consent of the author.


