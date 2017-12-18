

Cross-Layer Weighted Position-based Routing (CLWPR)
---------------------------------------------------

This model implements the base specification of the Cross-Layer 
Weighted Position-based Routing (CLWPR) protocol, which is a dynamic 
mobile ad hoc unicast routing protocol.  It has been developed at the
University of Surrey (UK) by Konstantinos Katsaros for NS-3 as part of
his PhD.

Model Description
*****************

The source code for the CLWPR model lives in the directory `src/clwpr`.

Design
++++++

Scope and Limitations
+++++++++++++++++++++

The model is for IPv4 only.  


References
++++++++++

.. 

Usage
*****

Examples
++++++++

Helpers
+++++++

A helper class for CLWPR has been written.  After an IPv4 topology
has been created and unique IP addresses assigned to each node, the
simulation script writer can call one of three overloaded functions
with different scope to enable CLWPR: ``ns3::ClwprHelper::Install
(NodeContainer container)``; ``ns3::ClwprHelper::Install (Ptr<Node>
node)``; or ``ns3::ClwprHelper::InstallAll (void)``

Attributes
++++++++++

In addition, the behavior of CLWPR can be modified by changing certain
attributes.  The method ``ns3::ClwprHelper::Set ()`` can be used
to set CLWPR attributes.  


Tracing
+++++++

Logging
+++++++

Caveats
+++++++

Validation
**********

Unit tests
++++++++++

Larger-scale performance tests
++++++++++++++++++++++++++++++

