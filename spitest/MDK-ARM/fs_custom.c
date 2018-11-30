#include "lfs.h"


#include "lwip/apps/httpd_opts.h"
#include "lwip/def.h"
#include "lwip/apps/fs.h"
#include "fsdata.h"
#include <string.h>

extern lfs_t lfs;

extern struct lfs_info info;

//extern lfs_file_t file;

int fs_open_custom(struct fs_file *file_fs, const char *name) {
	
	  lfs_file_t *file = malloc(sizeof(lfs_file_t));
	  memset(file,0,sizeof(lfs_file_t));
	  
	  const char *str;
	  str = name +1; 
    //printf("try open file name = %s \n\r", str);
    int resopen = lfs_file_open(&lfs, file, str, LFS_O_RDONLY);
	  //printf("resopen =  %d\n\r", resopen);
	  
    if (resopen < 0) {
			  free(file);
        return 0; 
    }
		
		lfs_stat(&lfs, str, &info);
		
		//printf ("try read stats file name from info = %s \n\r",info.name);
		file_fs->len = (int)info.size;
    file_fs->index = 0;
		file_fs->pextension = file;
		
		//printf("file opened size = %d\n\r", file_fs->len);
    
    return 1;
}

int fs_close_custom(struct fs_file *file_fs) {
	  lfs_file_t *file = (lfs_file_t*)file_fs->pextension;

    int resopen = lfs_file_close(&lfs, file);
	  //printf("try close file name = %s resopen = %d\n\r", file_fs->data, resopen);
    if (resopen < 0) {
        return 0; 
    }
		free(file);

    return 1;
}

int fs_read_custom(struct fs_file *file_fs, char *buffer, int count) {
	lfs_file_t *file = (lfs_file_t*)file_fs->pextension;
	int read = 0;
	//uint8_t lfs_read_buf[512] = {0};

  //printf("fs_read_custom try open read file name = %s count = %d \n\r", file_fs->data, count);
	if(file_fs->index == file_fs->len) {
    return FS_READ_EOF;
  }
	else {
		read = file_fs->len - file_fs->index;
    if(read > count) {
      read = count;
    }
		//if (read>512) read = 512;
		
		//printf("fs_read_custom read = %d \n\r", read);
		
		lfs_file_seek(&lfs, file,file_fs->index,LFS_SEEK_SET);
		read = lfs_file_read(&lfs, file, buffer, read);
		//MEMCPY(buffer,lfs_read_buf,read);
		//printf("try read = %d data from file = %s \n\r", read, lfs_read_buf);
	}
	file_fs->index += read;
  return read;
}

