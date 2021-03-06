// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver-info.h"

void callback(magenta_note_driver_t* dn, mx_bind_inst_t* bi, void* cookie) {
    printf("name:    %s\n", dn->name);
    printf("vendor:  %s\n", dn->vendor);
    printf("version: %s\n", dn->version);
    printf("binding:\n");

    char line[256];
    for (size_t n = 0; n < dn->bindcount; n++) {
        dump_bind_inst(bi + n, line, sizeof(line));
        printf("  %s\n", line);
    }
}

int main(int argc, char** argv) {
    while (argc > 1) {
        int fd;
        printf("[%s]\n", argv[1]);
        if ((fd = open(argv[1], O_RDONLY)) >= 0) {
            if (read_driver_info(fd, NULL, callback) < 0) {
                printf("error: no information found\n");
            }
            close(fd);
        } else {
            printf("error: cannot open file\n");
        }
        argc--;
        argv++;
    }
    return 0;
}
