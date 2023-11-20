# Digital-Twin Communication System

## Role of the Digital-Twin Communication System

The Digital-Twin Communication System (abbreviated as System in the following content) is responsible for providing a method of communication for the nodes in the network. The node can be a browser, a PC client program, or a microcontroller.&#x20;

The System is based on Ethernet architecture. End devices communicate over a wired or wireless Ethernet interface. Other components (such as the environmental Itsy-Bitsy microcontroller) also connect to devices that, in the end, communicate over Ethernet to the rest of the network.&#x20;

The goal of the System is to provide a unified and robust solution to run lab applications in a fair and consistent manner. Many protocols and procedures are put in place to ensure the System is operational and running to full capacity.



## Components

### Central Server

The central server receives and handles requests initiated by numerous clients in the network.

Users can attach custom functions to the server.

### Arduino Client



### Python Client



### Brower Client





## Data Sync

All the entries in the main database will be in the format of \[timestamp, key, value]

Because the System will use the timestamp as the unique primary key, which thus indicates that we can only have 1 entry per 1 ms, or 1kHz of data update rate across all clients. This should be sufficient to lower- to medium-scaled applications. For high data rate applications, a direct UDP method is preferred.



## Logging and Data Collection

The server can log certain entries from the database and keep them permanently in a separate database. This way, the storage space used by the main database can be minimized, and entries can be cleared in a FIFO manner.

The logging database is also equipped with query capabilities. When the client initiates a query request, the server will automatically route the request to the most suitable database to fetch data.

Below is a list of all data that is logged by the System:

| Entry                             | Unit        |
| --------------------------------- | ----------- |
| Temperature                       | ℃ (Celsius) |
| Humidity                          | %           |
| PM 1.0 concentration              |             |
| PM 2.5 concentration              |             |
| PM 10 concentration               |             |
| < 0.3 um particle count           | count       |
| < 1 um particle count             | count       |
| < 10 um particle count            | count       |
| Light intensity                   | lx (Lux)    |
| Magnetic field strength in X-axis |             |
| Magnetic field strength in Y-axis |             |
| Magnetic field strength in Z-axis |             |
| noise level                       | dB          |



## The Execution Process

### Environmental Monitoring



### Inventory Querying
