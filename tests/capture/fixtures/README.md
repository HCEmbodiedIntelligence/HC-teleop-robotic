# HC standalone regression inputs

hc-action-cases.json contains newly authored, explicit expected results for causality, receive visibility, partial axes, expiry, clock epochs, invalidation and on-change hold leases. It does not claim to reproduce the unavailable G0 cases.

hc-contract-baseline.json records hashes from committed migration baseline 39abb4c231d248140122dcb3872bf2f6bec2bf03, independently of the working implementation. Schema syntax is also checked. This checks preservation of HC migration contracts, not byte equivalence to an unavailable external G0 release.

Existing real recorder/raw/official LeRobot and uploader recovery tests remain the functional acceptance tests. Optional legacy uploader G0 tests still require their original fixtures and are not claimed to have passed.
