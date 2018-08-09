# criticalT
Reliability assessment and critical thread identification tool.

This tool is an extension of trace module provided by Simics toolset. trace module (simics/src/extensions/trace.c) provides both instruction and data trace of a program from Simics. 
criticalT extends the trace module by collecting information about the execution threads at runtime, and calculates vulnerability value and criticality degree value of each thread.

In order to use criticalT tool, Simics toolset needs to be configured for a new module, and the module should be activated for the target execution.
