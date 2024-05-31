#!/bin/bash

git checkout awf-latest
git pull
git checkout sugahara/working_branch
git merge awf-latest
