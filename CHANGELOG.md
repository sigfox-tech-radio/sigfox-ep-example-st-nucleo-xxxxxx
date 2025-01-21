# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [v2.0](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/releases/tag/v2.0) - 20 Jan 2025

### Changed

* Configure **SPI pins dynamically** to support multiple pinouts.
* Upgrade **sigfox-ep-lib** to [v4.0](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v4.0).
* Upgrade **sigfox-ep-addon-rfp** to [v2.0](https://github.com/sigfox-tech-radio/sigfox-ep-addon-rfp/releases/tag/v2.0).
* Upgrade **sigfox-ep-rf-api-st-s2lp** to [v4.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp/releases/tag/v4.0).
* Upgrade **sigfox-ep-rf-api-semtech-lr11xx** to [v3.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-semtech-lr11xx/releases/tag/v3.0).
* Upgrade **sigfox-ep-rf-api-semtech-sx126x** to [v2.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-semtech-sx126x/releases/tag/v2.0).
* Upgrade **LR11XX driver** to [v2.5.0](https://github.com/Lora-net/SWDR001/releases/tag/v2.5.0).
* Rename **lr1110dvk1tbks** shield by **lr1110mb1dis** 
* Rename **sx1261dvk1bas** shield by **sx1261mb1bas**

### Fixed

* Fix **STEVAL-FKI868V2** shield **default pinout**.

### Added

* New **modem** application with at-parser and addon RF&Protocol
* Support of **lr1110mb1djs** **lr1121mb1dis** **sx1261mb2bas** shields
* Create HW_API for every shields 

## [v1.0](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/releases/tag/v1.0) - 19 Jun 2024

### General

* First version of the Sigfox EP library implementation example on Nucleo board.

### Added

* Support of **Nucleo-L053R8** board.
* Support of **STEVAL-FKI868V2**, **LR1110DVK1TBKS** and **SX1261DVK1BAS** radio shields.
