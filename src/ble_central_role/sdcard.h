int mount_sdcard(void);

/**
 * @brief Get the disk status object
 * 
 * @return int DISK_STATUS_OK or other DISK_STATUS_*s
 * DISK_STATUS_OK = 0
 */
int get_disk_status();

int sdcard_mutext_init(struct k_mutex *sdcard_mutex);

extern struct k_mutex sdcard_mutex;