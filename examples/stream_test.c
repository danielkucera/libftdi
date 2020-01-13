/* stream_test.c
 *
 * Test reading  from FT2232H in synchronous FIFO mode.
 *
 * The FT2232H must supply data due to an appropriate circuit
 *
 * To check for skipped block with appended code, 
 *     a structure as follows is assumed
 * 1* uint32_t num (incremented in 0x4000 steps)
 * 3* uint32_t dont_care
 *
 * After start, data will be read in streaming until the program is aborted
 * Progess information wil be printed out
 * If a filename is given on the command line, the data read will be
 * written to that file
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <errno.h>
#include <ftdi.h>
#include <libusb.h>

typedef struct
{
    FTDIStreamCallback *callback;
    void *userdata;
    int packetsize;
    int activity;
    int result;
    FTDIProgressInfo progress;
} FTDIStreamState;

void check_outfile(char *);

static FILE *outputFile;

static int check = 1;
static int exitRequested = 0;
/*
 * sigintHandler --
 *
 *    SIGINT handler, so we can gracefully exit when the user hits ctrl-C.
 */

static void
sigintHandler(int signum)
{
   exitRequested = 1;
}

static void
usage(const char *argv0)
{
   fprintf(stderr,
           "Usage: %s [options...] \n"
           "Test streaming read from FT2232H\n"
           "[-P string] only look for product with given string\n"
           "[-n] don't check for special block structure\n"
           "\n"
           "If some filename is given, write data read to that file\n"
           "Progess information is printed each second\n"
           "Abort with ^C\n"
           "\n"
           "Options:\n"
           "\n"
           "Copyright (C) 2009 Micah Dowty <micah@navi.cx>\n"
           "Adapted for use with libftdi (C) 2010 Uwe Bonnes <bon@elektron.ikp.physik.tu-darmstadt.de>\n",
           argv0);
   exit(1);
}

static uint32_t start = 0;
static uint32_t offset = 0;
static uint64_t blocks = 0;
static uint32_t skips = 0;
static uint32_t n_err = 0;
static int
readCallback(uint8_t *buffer, int length, FTDIProgressInfo *progress, void *userdata)
{
   if (length)
   {
       if (check)
       {
           int i,rem;
           uint32_t num;
           for (i= offset; i<length-16; i+=16)
           {
               num = *(uint32_t*) (buffer+i);
               if (start && (num != start +0x4000))
               {
                   uint32_t delta = ((num-start)/0x4000)-1;
                   fprintf(stderr, "Skip %7d blocks from 0x%08x to 0x%08x at blocks %10llu\n",
                           delta, start -0x4000, num, (unsigned long long)blocks);
                   n_err++;
                   skips += delta;
               }
               blocks ++;
               start = num;
           }
           rem = length -i;
           if (rem >3)
           {
               num = *(uint32_t*) (buffer+i);
               if (start && (num != start +0x4000))
               {
                   uint32_t delta = ((num-start)/0x4000)-1;
                   fprintf(stderr, "Skip %7d blocks from 0x%08x to 0x%08x at blocks %10llu\n",
                           delta, start -0x4000, num, (unsigned long long) blocks);
                   n_err++;
                   skips += delta;
               }
               start = num;
           }
           else if (rem)
               start += 0x4000;
           if (rem != 0)
           {
               blocks ++;
               offset = 16-rem;
           }
       }
       if (outputFile)
       {
           if (fwrite(buffer, length, 1, outputFile) != 1)
           {
               perror("Write error");
               return 1;
           }
       }
   }
   if (progress)
   {
       fprintf(stderr, "%10.02fs total time %9.3f MiB captured %7.1f kB/s curr rate %7.1f kB/s totalrate %d dropouts\n",
               progress->totalTime,
               progress->current.totalBytes / (1024.0 * 1024.0),
               progress->currentRate / 1024.0,
               progress->totalRate / 1024.0,
               n_err);
   }
   return exitRequested ? 1 : 0;
}

static int
writeCallback(uint8_t *buffer, int length, FTDIProgressInfo *progress, void *userdata)
{
   if (outputFile)
   {
       if (feof(outputFile)) {
	       fseek (outputFile , 0 , SEEK_SET );
       }
       if (fread(buffer, length, 1, outputFile) < 1)
       {
           perror("File read error");
//           return 1;
       } else {
//           perror("File read OK");
       }
   }
/*
   for (int i=0; i<length; i++){
	   buffer[i] = 0xff;
   }
*/
   if (progress)
   {
       fprintf(stderr, "%10.02fs total time %9.3f MiB captured %7.1f kB/s curr rate %7.1f kB/s totalrate %d dropouts\n",
               progress->totalTime,
               progress->current.totalBytes / (1024.0 * 1024.0),
               progress->currentRate / 1024.0,
               progress->totalRate / 1024.0,
               n_err);
   }
   return exitRequested ? 1 : 0;
}

static double
TimevalDiff(const struct timeval *a, const struct timeval *b)
{
    return (a->tv_sec - b->tv_sec) + 1e-6 * (a->tv_usec - b->tv_usec);
}

static void LIBUSB_CALL
ftdi_writestream_cb(struct libusb_transfer *transfer)
{
    FTDIStreamState *state = transfer->user_data;
    int packet_size = state->packetsize;

    state->activity++;
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        int i;
        uint8_t *ptr = transfer->buffer;
        int length = transfer->actual_length;
        int numPackets = (length + packet_size - 1) / packet_size;
        int res = 0;

        for (i = 0; i < numPackets; i++)
        {
            int payloadLen;
            int packetLen = length;

            if (packetLen > packet_size)
                packetLen = packet_size;

            payloadLen = packetLen - 2;
            state->progress.current.totalBytes += payloadLen;

            res = state->callback(ptr, payloadLen,
                                  NULL, state->userdata);

            ptr += packetLen;
            length -= packetLen;
        }
        if (res)
        {
            free(transfer->buffer);
            libusb_free_transfer(transfer);
        }
        else
        {
            transfer->status = -1;
            state->result = libusb_submit_transfer(transfer);
        }
    }
    else
    {
        fprintf(stderr, "unknown status %d\n",transfer->status);
        state->result = LIBUSB_ERROR_IO;
    }
}

int
ftdi_writestream(struct ftdi_context *ftdi,
                FTDIStreamCallback *callback, void *userdata,
                int packetsPerTransfer, int numTransfers)
{
    struct libusb_transfer **transfers;
    FTDIStreamState state = { callback, userdata, ftdi->max_packet_size, 1 };
    int bufferSize = packetsPerTransfer * ftdi->max_packet_size;
    int xferIndex;
    int err = 0;

    /* Only FT2232H and FT232H know about the synchronous FIFO Mode*/
    if ((ftdi->type != TYPE_2232H) && (ftdi->type != TYPE_232H))
    {
        fprintf(stderr,"Device doesn't support synchronous FIFO mode\n");
        return 1;
    }

    /* We don't know in what state we are, switch to reset*/
    if (ftdi_set_bitmode(ftdi,  0xff, BITMODE_RESET) < 0)
    {
        fprintf(stderr,"Can't reset mode\n");
        return 1;
    }

    /* Purge anything remaining in the buffers*/
    if (ftdi_tcioflush(ftdi) < 0)
    {
        fprintf(stderr,"Can't flush FIFOs & buffers\n");
        return 1;
    }

    /*
     * Set up all transfers
     */

    transfers = calloc(numTransfers, sizeof *transfers);
    if (!transfers)
    {
        err = LIBUSB_ERROR_NO_MEM;
        goto cleanup;
    }

    for (xferIndex = 0; xferIndex < numTransfers; xferIndex++)
    {
        struct libusb_transfer *transfer;

        transfer = libusb_alloc_transfer(0);
        transfers[xferIndex] = transfer;
        if (!transfer)
        {
            err = LIBUSB_ERROR_NO_MEM;
            goto cleanup;
        }

        libusb_fill_bulk_transfer(transfer, ftdi->usb_dev, ftdi->in_ep,
                                  malloc(bufferSize), bufferSize,
                                  ftdi_writestream_cb,
                                  &state, 0);

        if (!transfer->buffer)
        {
            err = LIBUSB_ERROR_NO_MEM;
            goto cleanup;
        }

        transfer->status = -1;
        err = libusb_submit_transfer(transfer);
        if (err)
            goto cleanup;
    }

    /* Start the transfers only when everything has been set up.
     * Otherwise the transfers start stuttering and the PC not
     * fetching data for several to several ten milliseconds
     * and we skip blocks
     */
    if (ftdi_set_bitmode(ftdi,  0xff, BITMODE_SYNCFF) < 0)
    {
        fprintf(stderr,"Can't set synchronous fifo mode: %s\n",
                ftdi_get_error_string(ftdi));
        goto cleanup;
    }

    /*
     * Run the transfers, and periodically assess progress.
     */

    gettimeofday(&state.progress.first.time, NULL);

    do
    {
        FTDIProgressInfo  *progress = &state.progress;
        const double progressInterval = 1.0;
        struct timeval timeout = { 0, ftdi->usb_read_timeout * 1000};
        struct timeval now;

        int err = libusb_handle_events_timeout(ftdi->usb_ctx, &timeout);
        if (err ==  LIBUSB_ERROR_INTERRUPTED)
            /* restart interrupted events */
            err = libusb_handle_events_timeout(ftdi->usb_ctx, &timeout);
        if (!state.result)
        {
            state.result = err;
        }
        if (state.activity == 0)
            state.result = 1;
        else
            state.activity = 0;

        // If enough time has elapsed, update the progress
        gettimeofday(&now, NULL);
        if (TimevalDiff(&now, &progress->current.time) >= progressInterval)
        {
            progress->current.time = now;
            progress->totalTime = TimevalDiff(&progress->current.time,
                                              &progress->first.time);

            if (progress->prev.totalBytes)
            {
                // We have enough information to calculate rates

                double currentTime;

                currentTime = TimevalDiff(&progress->current.time,
                                          &progress->prev.time);

                progress->totalRate =
                    progress->current.totalBytes /progress->totalTime;
                progress->currentRate =
                    (progress->current.totalBytes -
                     progress->prev.totalBytes) / currentTime;
            }

            state.callback(NULL, 0, progress, state.userdata);
            progress->prev = progress->current;

        }
    } while (!state.result);

    /*
     * Cancel any outstanding transfers, and free memory.
     */

cleanup:
    fprintf(stderr, "cleanup\n");
    if (transfers)
        free(transfers);
    if (err)
        return err;
    else
        return state.result;
}


int main(int argc, char **argv)
{
   struct ftdi_context *ftdi;
   int err, c;
   FILE *of = NULL;
   char const *outfile  = 0;
   outputFile =0;
   exitRequested = 0;
   char *descstring = NULL;
   int option_index;
   static struct option long_options[] = {{NULL},};

   while ((c = getopt_long(argc, argv, "P:n", long_options, &option_index)) !=- 1)
       switch (c) 
       {
       case -1:
           break;
       case 'P':
           descstring = optarg;
           break;
       case 'n':
           check = 0;
           break;
       default:
           usage(argv[0]);
       }
   
   if (optind == argc - 1)
   {
       // Exactly one extra argument- a dump file
       outfile = argv[optind];
   }
   else if (optind < argc)
   {
       // Too many extra args
       usage(argv[0]);
   }
   
   if ((ftdi = ftdi_new()) == 0)
   {
       fprintf(stderr, "ftdi_new failed\n");
       return EXIT_FAILURE;
   }
   
   if (ftdi_set_interface(ftdi, INTERFACE_A) < 0)
   {
       fprintf(stderr, "ftdi_set_interface failed\n");
       ftdi_free(ftdi);
       return EXIT_FAILURE;
   }
   
   if (ftdi_usb_open_desc(ftdi, 0x0403, 0x6010, descstring, NULL) < 0)
   {
       fprintf(stderr,"Can't open ftdi device: %s\n",ftdi_get_error_string(ftdi));
       ftdi_free(ftdi);
       return EXIT_FAILURE;
   }
   
   /* A timeout value of 1 results in may skipped blocks */
   if(ftdi_set_latency_timer(ftdi, 2))
   {
       fprintf(stderr,"Can't set latency, Error %s\n",ftdi_get_error_string(ftdi));
       ftdi_usb_close(ftdi);
       ftdi_free(ftdi);
       return EXIT_FAILURE;
   }
   
/*   if(ftdi_usb_purge_rx_buffer(ftdi) < 0)
   {
       fprintf(stderr,"Can't rx purge\n",ftdi_get_error_string(ftdi));
       return EXIT_FAILURE;
       }*/
   if (outfile)
       if ((of = fopen(outfile,"r")) == 0)
           fprintf(stderr,"Can't open logfile %s, Error %s\n", outfile, strerror(errno));
   if (of)
       if (setvbuf(of, NULL, _IOFBF , 1<<16) == 0)
           outputFile = of;
   signal(SIGINT, sigintHandler);
   
//   err = ftdi_readstream(ftdi, readCallback, NULL, 8, 256);
   err = ftdi_writestream(ftdi, writeCallback, NULL, 8, 256);
   if (err < 0 && !exitRequested)
       exit(1);
   
   if (outputFile) {
       fclose(outputFile);
       outputFile = NULL;
   }
   fprintf(stderr, "Capture ended.\n");
   
   if (ftdi_set_bitmode(ftdi,  0xff, BITMODE_RESET) < 0)
   {
       fprintf(stderr,"Can't set synchronous fifo mode, Error %s\n",ftdi_get_error_string(ftdi));
       ftdi_usb_close(ftdi);
       ftdi_free(ftdi);
       return EXIT_FAILURE;
   }
   ftdi_usb_close(ftdi);
   ftdi_free(ftdi);
   signal(SIGINT, SIG_DFL);
   if (check && outfile)
   {
       if ((outputFile = fopen(outfile,"r")) == 0)
       {
           fprintf(stderr,"Can't open logfile %s, Error %s\n", outfile, strerror(errno));
           ftdi_usb_close(ftdi);
           ftdi_free(ftdi);
           return EXIT_FAILURE;
       }
       check_outfile(descstring);
       fclose(outputFile);
   }
   else if (check)
       fprintf(stderr,"%d errors of %llu blocks (%Le), %d (%Le) blocks skipped\n",
               n_err, (unsigned long long) blocks, (long double)n_err/(long double) blocks,
               skips, (long double)skips/(long double) blocks);
   exit (0);
}

void check_outfile(char *descstring)
{
    if(strcmp(descstring,"FT2232HTEST") == 0)
    {
       char buf0[1024];
       char buf1[1024];
       char bufr[1024];
       char *pa, *pb, *pc;
       unsigned int num_lines = 0, line_num = 1;
       int err_count = 0;
       unsigned int num_start, num_end;

       pa = buf0;
       pb = buf1;
       pc = buf0;
       if(fgets(pa, 1023, outputFile) == NULL)
       {
           fprintf(stderr,"Empty output file\n");
           return;
       }
       while(fgets(pb, 1023, outputFile) != NULL)
       {
           num_lines++;
           unsigned int num_save = num_start;
           if( sscanf(pa,"%6u%94s%6u",&num_start, bufr,&num_end) !=3)
           {
               fprintf(stdout,"Format doesn't match at line %8d \"%s",
                       num_lines, pa);
               err_count++;
               line_num = num_save +2;
           }
           else
           {
               if ((num_start+1)%100000 != num_end)
               {
                   if (err_count < 20)
                       fprintf(stdout,"Malformed line %d \"%s\"\n", 
                               num_lines, pa);
                   err_count++;
               }
               else if(num_start != line_num)
               {
                   if (err_count < 20)
                       fprintf(stdout,"Skipping from %d to %d\n", 
                               line_num, num_start);
                   err_count++;
                  
               }
               line_num = num_end;
           }
           pa = pb;
           pb = pc;
           pc = pa;
       }
       if(err_count)
           fprintf(stdout,"\n%d errors of %d data sets %f\n", err_count, num_lines, (double) err_count/(double)num_lines);
       else
           fprintf(stdout,"No errors for %d lines\n",num_lines);
   }
    else if(strcmp(descstring,"LLBBC10") == 0)
    { 
        uint32_t block0[4];
        uint32_t block1[4];
        uint32_t *pa = block0;
        uint32_t *pb = block1;
        uint32_t *pc = block0;
        uint32_t start= 0;
        uint32_t nread = 0;
        int n_shown = 0;
        int n_errors = 0;
        if (fread(pa, sizeof(uint32_t), 4,outputFile) < 4)
        {
            fprintf(stderr,"Empty result file\n");
            return;
        }
        while(fread(pb, sizeof(uint32_t), 4,outputFile) != 0)
        {
            blocks++;
            nread =  pa[0];
            if(start>0 && (nread != start))
            {
                if(n_shown < 30)
                {
                    fprintf(stderr, "Skip %7d blocks from 0x%08x to 0x%08x at blocks %10llu \n",
                            (nread-start)/0x4000, start -0x4000, nread, (unsigned long long) blocks);
                    n_shown ++;
                }
                n_errors++;
            }
            else if (n_shown >0) 
                n_shown--; 
            start = nread + 0x4000;
            pa = pb;
            pb = pc;
            pc = pa;
        }
        if(n_errors)
            fprintf(stderr, "%d blocks wrong from %llu blocks read\n",
                    n_errors, (unsigned long long) blocks);
        else
            fprintf(stderr, "%llu blocks all fine\n", (unsigned long long) blocks);
    }
}
