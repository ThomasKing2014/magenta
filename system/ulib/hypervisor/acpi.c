// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <unistd.h>

#include <hypervisor/acpi.h>
#include <hypervisor/ports.h>
#include <sys/stat.h>

#if __x86_64__
#include <acpica/acpi.h>
#include <acpica/actables.h>
#include <acpica/actypes.h>

static const char kDsdtPath[] = "/boot/data/dsdt.aml";
static const char kMadtPath[] = "/boot/data/madt.aml";

static uint8_t acpi_checksum(void* table, uint32_t length) {
    return UINT8_MAX - AcpiTbChecksum(table, length) + 1;
}

static void acpi_header(ACPI_TABLE_HEADER* header, const char* signature, uint32_t length) {
    memcpy(header->Signature, signature, ACPI_NAME_SIZE);
    header->Length = length;
    header->Checksum = acpi_checksum(header, header->Length);
}

static mx_status_t load_file(const char* path, uintptr_t addr, size_t size, uint32_t* actual) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open ACPI table \"%s\"\n", path);
        return ERR_IO;
    }
    struct stat stat;
    int ret = fstat(fd, &stat);
    if (ret < 0) {
        fprintf(stderr, "Failed to stat ACPI table \"%s\"\n", path);
        return ERR_IO;
    }
    if ((size_t)stat.st_size > size) {
        fprintf(stderr, "Not enough space for ACPI table \"%s\"\n", path);
        return ERR_IO;
    }
    ret = read(fd, (void*)addr, stat.st_size);
    if (ret < 0 || ret != stat.st_size) {
        fprintf(stderr, "Failed to read ACPI table \"%s\"\n", path);
        return ERR_IO;
    }
    *actual = stat.st_size;
    return NO_ERROR;
}
#endif // __x86_64__

mx_status_t guest_create_acpi_table(uintptr_t addr, size_t size, uintptr_t acpi_off) {
#if __x86_64__
    if (size < acpi_off + PAGE_SIZE)
        return ERR_BUFFER_TOO_SMALL;

    const uint32_t rsdt_entries = 2;
    const uint32_t rsdt_length = sizeof(ACPI_TABLE_RSDT) + (rsdt_entries - 1) * sizeof(uint32_t);

    // RSDP. ACPI 1.0.
    ACPI_RSDP_COMMON* rsdp = (ACPI_RSDP_COMMON*)(addr + acpi_off);
    ACPI_MAKE_RSDP_SIG(rsdp->Signature);
    memcpy(rsdp->OemId, "MX_HYP", ACPI_OEM_ID_SIZE);
    rsdp->RsdtPhysicalAddress = acpi_off + sizeof(ACPI_RSDP_COMMON);
    rsdp->Checksum = acpi_checksum(rsdp, ACPI_RSDP_CHECKSUM_LENGTH);

    // FADT.
    const uintptr_t fadt_off = rsdp->RsdtPhysicalAddress + rsdt_length;
    ACPI_TABLE_FADT* fadt = (ACPI_TABLE_FADT*)(addr + fadt_off);
    const uintptr_t dsdt_off = fadt_off + sizeof(ACPI_TABLE_FADT);
    fadt->Dsdt = dsdt_off;
    fadt->Pm1aEventBlock = PM1_EVENT_PORT;
    fadt->Pm1EventLength = (ACPI_PM1_REGISTER_WIDTH / 8) * 2 /* enable and status registers */;
    fadt->Pm1aControlBlock = PM1_CONTROL_PORT;
    fadt->Pm1ControlLength = ACPI_PM1_REGISTER_WIDTH / 8;
    acpi_header(&fadt->Header, ACPI_SIG_FADT, sizeof(ACPI_TABLE_FADT));

    // DSDT.
    uint32_t actual;
    mx_status_t status = load_file(kDsdtPath, addr + dsdt_off, size - dsdt_off, &actual);
    if (status != NO_ERROR)
        return status;

    // MADT.
    const uintptr_t madt_off = dsdt_off + actual;
    status = load_file(kMadtPath, addr + madt_off, size - madt_off, &actual);
    if (status != NO_ERROR)
        return status;

    // RSDT.
    ACPI_TABLE_RSDT* rsdt = (ACPI_TABLE_RSDT*)(addr + rsdp->RsdtPhysicalAddress);
    rsdt->TableOffsetEntry[0] = fadt_off;
    rsdt->TableOffsetEntry[1] = madt_off;
    acpi_header(&rsdt->Header, ACPI_SIG_RSDT, rsdt_length);
    return NO_ERROR;
#else // __x86_64__
    return ERR_NOT_SUPPORTED;
#endif // __x86_64__
}
