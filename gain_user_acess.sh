#!/bin/bash

set -e


sudo usermod -aG video "$USER"
bash -l
groups