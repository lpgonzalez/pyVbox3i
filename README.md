## Welcome to pyVbox3i GitHub Page

This is a `python` project to handle [Racelogic VBOX 3i](https://www.vboxautomotive.co.uk/index.php/en/products/data-loggers/vbox-3i) real time captured data via **COM (serial)** communication. This helps to integrate VBOX 3i in a variety of projects where it is required to process high precision data in real time.

One of the main problems we faced with Racelogic VBOX 3i is that there is no SDK or API to access the data that it is capturing from the different sensors, however, in [this website](https://racelogic.support/01VBOX_Automotive/01VBOX_data_loggers/VBOX_3i_Range/VBOX_3i_User_Manual_(All_Variants)/15_-_VB3i_Technical_Properties/VB3i_CAN_Output) they describe the message format that is transmitted via CAN bus, and that can be accesed via USB port (and virtual COM interface).
