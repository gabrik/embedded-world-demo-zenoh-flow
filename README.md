# Zenoh-Flow Embedded World Demo

This repository contains the Zenoh Flow demo for Embedded World 2023

## How to run

First install [Zenoh](https://github.com/eclipse-zenoh/zenoh#how-to-install-it), [Zenoh-Flow](https://github.com/eclipse-zenoh/zenoh-flow/wiki/Installation-(v0.4.0)#building) and the [Zenoh-Flow python plugin](https://github.com/eclipse-zenoh/zenoh-flow-python/#how-to-build).

Then open a terminal window and start zenoh with the zenoh-flow plugin `./start-zenoh.sh`

Open another terminal window and use the `zfctl` to launch the flow `zfctl launch distance-alert.yml`
