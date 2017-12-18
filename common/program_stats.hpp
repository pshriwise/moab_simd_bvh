#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>

// function for reporting the current memory usage of the test
void report_memory_usage()
{
  struct rusage r_usage;
  getrusage(RUSAGE_SELF, &r_usage);

  // try going to /proc to estimate total memory
  char file_str[4096], dum_str[4096];
  int file_ptr = open("/proc/self/stat", O_RDONLY);
  int file_len = read(file_ptr, file_str, sizeof(file_str)-1);
  if (file_len == 0) {
    close(file_ptr);
    return;
  }

  close(file_ptr);
  file_str[file_len] = '\0';
  // read the preceeding fields and the ones we really want...
  int dum_int;
  unsigned int dum_uint, vm_size, rss;
  int num_fields = sscanf(file_str,
			  "%d " // pid
			  "%s " // comm
			  "%c " // state
			  "%d %d %d %d %d " // ppid, pgrp, session, tty, tpgid
			  "%u %u %u %u %u " // flags, minflt, cminflt, majflt, cmajflt
			  "%d %d %d %d %d %d " // utime, stime, cutime, cstime, counter, priority
			  "%u %u " // timeout, itrealvalue
			  "%d " // starttime
			  "%u %u", // vsize, rss
			  &dum_int,
			  dum_str,
			  dum_str,
			  &dum_int, &dum_int, &dum_int, &dum_int, &dum_int,
			  &dum_uint, &dum_uint, &dum_uint, &dum_uint, &dum_uint,
			  &dum_int, &dum_int, &dum_int, &dum_int, &dum_int, &dum_int,
			  &dum_uint, &dum_uint,
			  &dum_int,
			  &vm_size, &rss);
  if (num_fields == 24) {
    std::cout << "Current memory usage in bytes: " << ((double)vm_size)
	      << " (" << ((double)vm_size)/(1024*1024) << " MB)" << std::endl;
  }
  else {
    std::cout << "Could not retrieve memory usage" << std::endl;
  }

}
