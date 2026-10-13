#!/usr/bin/env bash
rsync -av --exclude='/.git' --filter=':- .gitignore' ./ root@192.168.42.101:Bela/projects/RiseTogether-bela-forwarder
