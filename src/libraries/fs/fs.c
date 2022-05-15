/*
 * fs.c
 *
 *  Created on: 06.01.2019
 *      Author: Erich Styger
 */
#include "fs.h"
#include "./w25qxx/w25qxx.h"
#include "./littleFS/lfs.h"
#include "src/libraries/printf/printf.h"

#define std_printf printf_

#define FS_FILE_NAME_SIZE  32 /* Length of file name, used in buffers */

/* variables used by the file system */
static bool FS_isMounted = FALSE;
static lfs_t FS_lfs;

static int block_device_read(const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size) {
  uint8_t res;

  res = W25_Read(block * c->block_size + off, buffer, size);
  if (res != FS_ERR_OK) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
  uint8_t res;

  res = W25_ProgramPage(block * c->block_size + off, buffer, size);
  if (res != FS_ERR_OK) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block) {
  uint8_t res;

  res = W25_EraseSector4K(block * c->block_size);
  if (res != FS_ERR_OK) {
    return LFS_ERR_IO;
  }
  return LFS_ERR_OK;
}

int block_device_sync(const struct lfs_config *c) {
  return LFS_ERR_OK;
}

// configuration of the file system is provided by this struct
const struct lfs_config FS_cfg = {
  // block device operations
  .read  = block_device_read,
  .prog  = block_device_prog,
  .erase = block_device_erase,
  .sync  = block_device_sync,

  .read_size = 256,
  .prog_size = 256,
  .block_size = 4096,
  .block_count = 4096,
  .block_cycles = 500,
  .cache_size = 256,
  .lookahead_size = 512,
  
};

uint8_t FS_Format(void) {
  int res;

  if (FS_isMounted) {
    // CLS1_SendStr("File system is already mounted, unmount it first.\r\n", io->stdErr);
    std_printf("File system is already mounted, unmount it first.\r\n");
    return FS_ERR_FAILED;
  }
  // if (io!=NULL) {
  //   CLS1_SendStr("Formatting ...", io->stdOut);
  // }
  std_printf("Formatting ...");
  res = lfs_format(&FS_lfs, &FS_cfg);
  if (res == LFS_ERR_OK) {
    // if (io!=NULL) {
    //   CLS1_SendStr(" done.\r\n", io->stdOut);
    // }
    std_printf(" done.\r\n");
    return FS_ERR_OK;
  } else {
    // if (io!=NULL) {
    //   CLS1_SendStr(" FAILED!\r\n", io->stdErr);
    // }
    std_printf(" FAILED!\r\n");
    return FS_ERR_FAILED;
  }
}

uint8_t FS_Mount(void) {
  int res;

  if (FS_isMounted) {
    // if (io!=NULL) {
    //   // CLS1_SendStr("File system is already mounted.\r\n", io->stdErr);
    // }
    std_printf("File system is already mounted, unmount it first.\r\n");
    return FS_ERR_FAILED;
  }
  // if (io!=NULL) {
  //   CLS1_SendStr("Mounting ...", io->stdOut);
  // }
  std_printf("Mounting ...");
  res = lfs_mount(&FS_lfs, &FS_cfg);
  if (res == LFS_ERR_OK) {
    // if (io!=NULL) {
    //   CLS1_SendStr(" done.\r\n", io->stdOut);
    // }
    std_printf(" done.\r\n");
    FS_isMounted = TRUE;
    return FS_ERR_OK;
  } else {
    // if (io!=NULL) {
    //   CLS1_SendStr(" FAILED!\r\n", io->stdErr);
    // }
    std_printf(" FAILED!\r\n");
    return FS_ERR_FAILED;
  }
}

uint8_t FS_Unmount(void) {
  int res;

  if (!FS_isMounted) {
    // if (io!=NULL) {
    //   CLS1_SendStr("File system is already unmounted.\r\n", io->stdErr);
    // }
    std_printf("File system is already unmounted.\r\n");
    return FS_ERR_FAILED;
  }
  // if (io!=NULL) {
  //   CLS1_SendStr("Unmounting ...", io->stdOut);
  // }
  std_printf("Unmounting ...");
  res = lfs_unmount(&FS_lfs);
  if (res == LFS_ERR_OK) {
    // if (io!=NULL) {
    //   CLS1_SendStr(" done.\r\n", io->stdOut);
    // }
    std_printf(" done.\r\n");
    FS_isMounted = FALSE;
    return FS_ERR_OK;
  } else {
    // if (io!=NULL) {
    //   CLS1_SendStr(" FAILED!\r\n", io->stdErr);
    // }
    std_printf(" FAILED!\r\n");
    return FS_ERR_FAILED;
  }
}

// uint8_t FS_Dir(const char *path, CLS1_ConstStdIOType *io) {
uint8_t FS_Dir(const char *path) {
  int res;
  lfs_dir_t dir;
  struct lfs_info info;

  if (!FS_isMounted) {
    std_printf("File system is not mounted, mount it first.\r\n");
    return FS_ERR_FAILED;
  }
  if (path == NULL) {
    path = "/"; /* default path */
  }
  res = lfs_dir_open(&FS_lfs, &dir, path);
  if (res != LFS_ERR_OK) {
    std_printf("FAILED lfs_dir_open()!\r\n");
    return FS_ERR_FAILED;
  }
  for (;;) {
    res = lfs_dir_read(&FS_lfs, &dir, &info);
    if (res < 0) {
      std_printf("FAILED lfs_dir_read()!\r\n");
      return FS_ERR_FAILED;
    }
    if (res == 0) { /* no more files */
      break;
    }
    switch (info.type) {
    case LFS_TYPE_REG: std_printf("reg "); break;
    case LFS_TYPE_DIR: std_printf("dir "); break;
    default:           std_printf("?   "); break;
    }
    static const char *prefixes[] = {"", "K", "M", "G"}; /* prefixes for kilo, mega and giga */
    for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
      if (info.size >= (1 << 10 * i) - 1) {
        std_printf("%*u%sB ", 4 - (i != 0), info.size >> 10 * i, prefixes[i]);
        break;
      }
    } /* for */
    std_printf(info.name);
    std_prinf("\r\n");
  } /* for */
  res = lfs_dir_close(&FS_lfs, &dir);
  if (res != LFS_ERR_OK) {
    std_prinf("FAILED lfs_dir_close()!\r\n");
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_CopyFile(const char *srcPath, const char *dstPath, CLS1_ConstStdIOType *io) {
uint8_t FS_CopyFile(const char *srcPath, const char *dstPath) {
  lfs_file_t fsrc, fdst;
  uint8_t res =  FS_ERR_OK;
  int result, nofBytesRead;
  uint8_t buffer[32];   /* copy buffer */

  if (!FS_isMounted) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    std_printf("ERROR: File system is not mounted.\r\n");
    return FS_ERR_FAILED;
  }

  /* open source file */
  result = lfs_file_open(&FS_lfs, &fsrc, srcPath, LFS_O_RDONLY);
  if (result < 0) {
    // CLS1_SendStr((const unsigned char*)"*** Failed opening source file!\r\n", io->stdErr);
    std_printf("*** Failed opening source file!\r\n");
    return FS_ERR_FAILED;
  }
  /* create destination file */
  result = lfs_file_open(&FS_lfs, &fdst, dstPath, LFS_O_WRONLY | LFS_O_CREAT);
  if (result < 0) {
    lfs_file_close(&FS_lfs, &fsrc);
    // CLS1_SendStr((const unsigned char*)"*** Failed opening destination file!\r\n", io->stdErr);
    std_printf("*** Failed opening destination file!\r\n");
    return FS_ERR_FAILED;
  }
  /* now copy source to destination */
  for (;;) {
    nofBytesRead = lfs_file_read(&FS_lfs, &fsrc, buffer, sizeof(buffer));
    if (nofBytesRead < 0) {
      // CLS1_SendStr((const unsigned char*)"*** Failed reading source file!\r\n", io->stdErr);
      std_printf("*** Failed reading source file!\r\n");
      res = FS_ERR_FAILED;
      break;
    }
    if (nofBytesRead == 0) { /* end of file */
      break;
    }
    result = lfs_file_write(&FS_lfs, &fdst, buffer, nofBytesRead);
    if (result < 0) {
      // CLS1_SendStr((const unsigned char*)"*** Failed writing destination file!\r\n", io->stdErr);
      std_printf("*** Failed writing destination file!\r\n");
      res = FS_ERR_FAILED;
      break;
    }
  } /* for */
  /* close all files */
  result = lfs_file_close(&FS_lfs, &fsrc);
  if (result < 0) {
    // CLS1_SendStr((const unsigned char*)"*** Failed closing source file!\r\n", io->stdErr);
    std_printf("*** Failed closing source file!\r\n");
    res = FS_ERR_FAILED;
  }
  result = lfs_file_close(&FS_lfs, &fdst);
  if (result < 0) {
    // CLS1_SendStr((const unsigned char*)"*** Failed closing destination file!\r\n", io->stdErr);
    std_printf("*** Failed closing destination file!\r\n");
    res = FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_MoveFile(const char *srcPath, const char *dstPath, CLS1_ConstStdIOType *io) {
uint8_t FS_MoveFile(const char *srcPath, const char *dstPath) {
  if (!FS_isMounted) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    std_printf("ERROR: File system is not mounted.\r\n");
    return FS_ERR_FAILED;
  }
  if (lfs_rename(&FS_lfs, srcPath, dstPath) < 0) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: failed renaming file or directory.\r\n", io->stdErr);
    // }
    std_printf("ERROR: failed renaming file or directory.\r\n");
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

static uint8_t readFromFile(void *hndl, uint32_t addr, uint8_t *buf, size_t bufSize) {
  lfs_file_t *fp;

  fp = (lfs_file_t*)hndl;
  if (lfs_file_read(&FS_lfs, fp, buf, bufSize) < 0) {
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_PrintHexFile(const char *filePath, CLS1_ConstStdIOType *io) {
uint8_t FS_PrintHexFile(const char *filePath) {
  lfs_file_t file;
  uint8_t res = FS_ERR_OK;
  int32_t fileSize;
  int result;

  if (!FS_isMounted) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    std_printf("ERROR: File system is not mounted.\r\n");
    return FS_ERR_FAILED;
  }
  result = lfs_file_open(&FS_lfs, &file, filePath, LFS_O_RDONLY);
  if (result < 0) {
    // if (io!=NULL) {
    //   CLS1_SendStr("ERROR: Failed opening file.\r\n", io->stdErr);
    // }
    std_printf("ERROR: Failed opening file.\r\n");
    return FS_ERR_FAILED;
  }
  fileSize = lfs_file_size(&FS_lfs, &file);
  if (fileSize < 0) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: getting file size\r\n", io->stdErr);
    //   (void)lfs_file_close(&FS_lfs, &file);
    // }
    std_printf("ERROR: getting file size\r\n");
    lfs_file_close(&FS_lfs, &file);
    return FS_ERR_FAILED;
  }
  res = CLS1_PrintMemory(&file, 0, fileSize - 1, 4, 16, readFromFile);
  if (res != FS_ERR_OK) {
    // CLS1_SendStr("ERROR while calling PrintMemory()\r\n", io->stdErr);
    std_printf("ERROR while calling PrintMemory()\r\n");
  }
  lfs_file_close(&FS_lfs, &file);
  return res;
}

// uint8_t FS_RemoveFile(const char *filePath, CLS1_ConstStdIOType *io) {
uint8_t FS_RemoveFile(const char *filePath) {
  int result;

  if (!FS_isMounted) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    std_printf("ERROR: File system is not mounted.\r\n");
    return FS_ERR_FAILED;
  }
  result = lfs_remove(&FS_lfs, filePath);
  if (result < 0) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: Failed removing file.\r\n", io->stdErr);
    // }
    std_printf("ERROR: Failed removing file.\r\n");
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

// uint8_t FS_RunBenchmark(CLS1_ConstStdIOType *io) {
uint8_t FS_RunBenchmark(void) {
  lfs_file_t file;
  int result;
  uint32_t i;
  uint8_t read_buf[10];
  // TIMEREC time, startTime;
  uint32_t time, startTime;
  int32_t start_mseconds, mseconds;

  if (!FS_isMounted) {
    // if (io != NULL) {
    //   CLS1_SendStr("ERROR: File system is not mounted.\r\n", io->stdErr);
    // }
    std_printf("ERROR: File system is not mounted.\r\n");
    return FS_ERR_FAILED;
  }
  /* write benchmark */
  // CLS1_SendStr((const unsigned char*)"Benchmark: write/copy/read a 100kB file:\r\n", io->stdOut);
  // CLS1_SendStr((const unsigned char*)"Delete existing benchmark files...\r\n", io->stdOut);
  std_printf("Benchmark: write/copy/read a 100kB file:\r\n");
  std_printf("Delete existing benchmark files...\r\n");
  FS_RemoveFile("./bench.txt");
  FS_RemoveFile("./copy.txt");

  // CLS1_SendStr((const unsigned char*)"Create benchmark file...\r\n", io->stdOut);
  std_printf("Create benchmark file...\r\n");
  // (void)TmDt1_GetTime(&startTime);
  startTime = millis();
  if (lfs_file_open(&FS_lfs, &file, "./bench.txt", LFS_O_WRONLY | LFS_O_CREAT)<0) {
    // CLS1_SendStr((const unsigned char*)"*** Failed creating benchmark file!\r\n", io->stdErr);
    std_printf("*** Failed creating benchmark file!\r\n");
    return FS_ERR_FAILED;
  }
  for (i = 0; i < 10240; i++) {
    if (lfs_file_write(&FS_lfs, &file, "benchmark ", sizeof("benchmark ") - 1) < 0) {
      // CLS1_SendStr((const unsigned char*)"*** Failed writing file!\r\n", io->stdErr);
      std_printf("*** Failed writing file!\r\n");
      lfs_file_close(&FS_lfs, &file);
      return FS_ERR_FAILED;
    }
  }
  lfs_file_close(&FS_lfs, &file);
//   (void)TmDt1_GetTime(&time);
//   start_mseconds = startTime.Hour*60*60*1000 + startTime.Min*60*1000 + startTime.Sec*1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//   + startTime.Sec100*10
// #endif
//   ;
//   mseconds = time.Hour*60*60*1000 + time.Min*60*1000 + time.Sec*1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//   + time.Sec100*10
// #endif
//   - start_mseconds;
  mseconds = millis() - startTime;
  // CLS1_SendNum32s(mseconds, io->stdOut);
  std_printf("%u", mseconds);
  // CLS1_SendStr((const unsigned char*)" ms for writing (", io->stdOut);
  std_printf(" ms for writing (");
  // CLS1_SendNum32s((100 * 1000) / mseconds, io->stdOut);
  std_printf("%u", (100 * 1000) / mseconds);
  // CLS1_SendStr((const unsigned char*)" kB/s)\r\n", io->stdOut);
  std_printf(" kB/s)\r\n");

  /* read benchmark */
  // CLS1_SendStr((const unsigned char*)"Read 100kB benchmark file...\r\n", io->stdOut);
  std_printf("Read 100kB benchmark file...\r\n");
  // (void)TmDt1_GetTime(&startTime);
  startTime = millis();
  if (lfs_file_open(&FS_lfs, &file, "./bench.txt", LFS_O_RDONLY) < 0) {
    // CLS1_SendStr((const unsigned char*)"*** Failed opening benchmark file!\r\n", io->stdErr);
    std_printf("*** Failed opening benchmark file!\r\n");
    return FS_ERR_FAILED;
  }
  for (i = 0; i < 10240; i++) {
    if (lfs_file_read(&FS_lfs, &file, &read_buf[0], sizeof(read_buf)) < 0) {
      // CLS1_SendStr((const unsigned char*)"*** Failed reading file!\r\n", io->stdErr);
      std_printf("*** Failed reading file!\r\n");
      lfs_file_close(&FS_lfs, &file);
      return FS_ERR_FAILED;
    }
  }
  lfs_file_close(&FS_lfs, &file);
//   (void)TmDt1_GetTime(&time);
//   start_mseconds = startTime.Hour * 60 * 60 * 1000 + startTime.Min * 60 * 1000 + startTime.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//                    + startTime.Sec100 * 10
// #endif
//                    ;
//   mseconds = time.Hour * 60 * 60 * 1000 + time.Min * 60 * 1000 + time.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//              + time.Sec100 * 10
// #endif
//              - start_mseconds;
  mseconds = millis() - startTime;
  // CLS1_SendNum32s(mseconds, io->stdOut);
  std_printf("%u", mseconds);
  // CLS1_SendStr((const unsigned char*)" ms for reading (", io->stdOut);
  std_printf(" ms for reading (");
  // CLS1_SendNum32s((100 * 1000) / mseconds, io->stdOut);
  std_printf("%u", (100 * 1000) / mseconds);
  // CLS1_SendStr((const unsigned char*)" kB/s)\r\n", io->stdOut);
  std_printf(" kB/s)\r\n");

  /* copy benchmark */
  // CLS1_SendStr((const unsigned char*)"Copy 100kB file...\r\n", io->stdOut);
  std_printf("Copy 100kB file...\r\n");
  // (void)TmDt1_GetTime(&startTime);
  startTime = millis();
  FS_CopyFile((const unsigned char*)"./bench.txt", (const unsigned char*)"./copy.txt");
//   (void)TmDt1_GetTime(&time);
//   start_mseconds = startTime.Hour * 60 * 60 * 1000 + startTime.Min * 60 * 1000 + startTime.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//                    + startTime.Sec100 * 10
// #endif
//                    ;
//   mseconds = time.Hour * 60 * 60 * 1000 + time.Min * 60 * 1000 + time.Sec * 1000
// #if TmDt1_HAS_SEC100_IN_TIMEREC
//              + time.Sec100 * 10
// #endif
//              - start_mseconds;
  // CLS1_SendNum32s(mseconds, io->stdOut);
  std_printf("%u", mseconds);
  // CLS1_SendStr((const unsigned char*)" ms for copy (", io->stdOut);
  std_printf(" ms for copy (");
  // CLS1_SendNum32s((100 * 1000) / mseconds, io->stdOut);
  std_printf("%u", (100 * 1000) / mseconds);
  // CLS1_SendStr((const unsigned char*)" kB/s)\r\n", io->stdOut);
  std_printf(" kB/s)\r\n");
  // CLS1_SendStr((const unsigned char*)"done!\r\n", io->stdOut);
  std_printf("done!\r\n");
  return FS_ERR_OK;
}

// static uint8_t FS_PrintStatus(CLS1_ConstStdIOType *io) {
static uint8_t FS_PrintStatus(void) {
  uint8_t buf[24];

  // CLS1_SendStatusStr((const unsigned char*)"FS", (const unsigned char*)"\r\n", io->stdOut);
  // CLS1_SendStatusStr((const unsigned char*)"  mounted", FS_isMounted ? "yes\r\n" : "no\r\n", io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count * FS_cfg.block_size);
  // UTIL1_strcat(buf, sizeof(buf), " bytes\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  space", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.read_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  read_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.prog_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  prog_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_size);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  block_size", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.block_count);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  block_count", buf, io->stdOut);

  // UTIL1_Num32uToStr(buf, sizeof(buf), FS_cfg.lookahead);
  // UTIL1_strcat(buf, sizeof(buf), "\r\n");
  // CLS1_SendStatusStr((const unsigned char*)"  lookahead", buf, io->stdOut);
  return FS_ERR_OK;
}

#if 0

uint8_t FS_ParseCommand(const unsigned char* cmd, bool *handled, const CLS1_StdIOType *io) {
  unsigned char fileNameSrc[FS_FILE_NAME_SIZE], fileNameDst[FS_FILE_NAME_SIZE];
  size_t lenRead;

  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP) == 0 || UTIL1_strcmp((char*)cmd, "FS help") == 0) {
    CLS1_SendHelpStr((unsigned char*)"FS", (const unsigned char*)"Group of FS commands\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  help|status", (const unsigned char*)"Print help or status information\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  format", (const unsigned char*)"Format the file system\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  mount", (const unsigned char*)"Mount the file system\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  unmount", (const unsigned char*)"unmount the file system\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  ls", (const unsigned char*)"List directory and files\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  rm <file>", (const unsigned char*)"Remove a file\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  mv <src> <dst>", (const unsigned char*)"Rename a file\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  cp <src> <dst>", (const unsigned char*)"Copy a file\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  printhex <file>", (const unsigned char*)"Print the file data in hexadecimal format\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  benchmark", (const unsigned char*)"Run a benchmark to measure performance\r\n", io->stdOut);
    *handled = TRUE;
    return FS_ERR_OK;
  } else if (UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS) == 0 || UTIL1_strcmp((char*)cmd, "FS status") == 0) {
    *handled = TRUE;
    return FS_PrintStatus(io);
  } else if (UTIL1_strcmp((char*)cmd, "FS format") == 0) {
    *handled = TRUE;
    return FS_Format(io);
  } else if (UTIL1_strcmp((char*)cmd, "FS mount") == 0) {
    *handled = TRUE;
    return FS_Mount(io);
  } else if (UTIL1_strcmp((char*)cmd, "FS unmount") == 0) {
    *handled = TRUE;
    return FS_Unmount(io);
  } else if (UTIL1_strcmp((char*)cmd, "FS ls") == 0) {
    *handled = TRUE;
    return FS_Dir(NULL, io);
  } else if (UTIL1_strcmp((char*)cmd, "FS benchmark") == 0) {
    *handled = TRUE;
    return FS_RunBenchmark(io);
  } else if (UTIL1_strncmp((char*)cmd, "FS printhex ", sizeof("FS printhex ") - 1) == 0) {
    *handled = TRUE;
    if (   (UTIL1_ReadEscapedName(cmd + sizeof("FS printhex ") - 1, fileNameSrc,
                                  sizeof(fileNameSrc), &lenRead, NULL, NULL) == FS_ERR_OK)
       )
    {
      return FS_PrintHexFile(fileNameSrc, io);
    }
    return FS_ERR_FAILED;
  } else if (UTIL1_strncmp((char*)cmd, "FS rm ", sizeof("FS rm ") - 1) == 0) {
    *handled = TRUE;
    if (   (UTIL1_ReadEscapedName(cmd + sizeof("FS rm ") - 1, fileNameSrc,
                                  sizeof(fileNameSrc), &lenRead, NULL, NULL) == FS_ERR_OK)
       )
    {
      return FS_RemoveFile(fileNameSrc, io);
    }
    return FS_ERR_FAILED;
  } else if (UTIL1_strncmp((char*)cmd, "FS mv ", sizeof("FS mv ") - 1) == 0) {
    *handled = TRUE;
    if (   (UTIL1_ReadEscapedName(cmd + sizeof("FS mv ") - 1, fileNameSrc,
                                  sizeof(fileNameSrc), &lenRead, NULL, NULL) == FS_ERR_OK)
           && *(cmd + sizeof("FS cp ") - 1 + lenRead) == ' '
           && (UTIL1_ReadEscapedName(cmd + sizeof("FS mv ") - 1 + lenRead + 1,
                                     fileNameDst,
                                     sizeof(fileNameDst), NULL, NULL, NULL) == FS_ERR_OK)
       )
    {
      return FS_MoveFile(fileNameSrc, fileNameDst, io);
    }
    return FS_ERR_FAILED;
  } else if (UTIL1_strncmp((char*)cmd, "FS cp ", sizeof("FS cp ") - 1) == 0) {
    *handled = TRUE;
    if (   (UTIL1_ReadEscapedName(cmd + sizeof("FS cp ") - 1, fileNameSrc,
                                  sizeof(fileNameSrc), &lenRead, NULL, NULL) == FS_ERR_OK)
           && *(cmd + sizeof("FS cp ") - 1 + lenRead) == ' '
           && (UTIL1_ReadEscapedName(cmd + sizeof("FS cp ") - 1 + lenRead + 1,
                                     fileNameDst,
                                     sizeof(fileNameDst), NULL, NULL, NULL) == FS_ERR_OK)
       )
    {
      return FS_CopyFile(fileNameSrc, fileNameDst, io);
    }
    return FS_ERR_FAILED;
  }
  return FS_ERR_OK;
}

#endif

uint8_t FS_Init(void) {
  if (W25_Init() != W25_ERR_OK) {
    return FS_ERR_FAILED;
  }

  std_printf("Attempting to mount existing media\r\n");
  if (FS_Mount() != FS_ERR_OK) {
    std_printf("Could not mount media, attemping to format\r\n");
    if (FS_Format() != FS_ERR_OK) {
      std_printf("Format failed\r\n");
      return FS_ERR_FAILED;
    }
    std_printf("Attempting to mount freshly formatted media\r\n");
    if (FS_Mount() != FS_ERR_OK) {
      std_printf("Mount after format failed\r\n");
      return FS_ERR_FAILED;
    }
  }

  return FS_ERR_OK;
}
