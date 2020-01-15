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

static FILE *outputFile;
static FILE *inputFile;

//TODO: proper definition
#define WRITE 0
#define READ 1

int mode = WRITE;

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
	   "[-r] read data from file, write to FT (default: vice-versa)\n"
           "\n"
           "If some filename is given, use that file for reading/writing data\n"
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

static uint32_t n_err = 0;
static int
callback(uint8_t *buffer, int length, FTDIProgressInfo *progress, void *userdata)
{
   if (length)
   {
       if (outputFile)
       {
           if (fwrite(buffer, length, 1, outputFile) != 1)
           {
               perror("Write error");
               return 1;
           }
       }
   }
   if (inputFile)
   {
       if (feof(outputFile)) {
	       fseek (outputFile , 0 , SEEK_SET );
       }
       if (fread(buffer, length, 1, outputFile) < 1)
       {
           perror("File read error");
           return 1;
       } else {
//           perror("File read OK");
       }
   }
   if (progress)
   {
       fprintf(stderr, "%10.02fs total time %9.3f MiB transferred %7.1f kB/s curr rate %7.1f kB/s totalrate %d dropouts\n",
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
ftdi_stream_cb(struct libusb_transfer *transfer)
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
ftdi_stream(struct ftdi_context *ftdi,
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

        libusb_fill_bulk_transfer(transfer, ftdi->usb_dev, 
                                  (mode == WRITE) ? ftdi->out_ep : ftdi->in_ep,
                                  //TODO: don't send empty buffers in first batch
                                  malloc(bufferSize), bufferSize,
                                  ftdi_stream_cb,
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
   FILE *df = NULL;
   char const *file  = 0;
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
       case 'r':
           mode = READ;
           break;
       default:
           usage(argv[0]);
       }
   
   if (optind == argc - 1)
   {
       // Exactly one extra argument- a dump file
       file = argv[optind];
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
   if (file){
       if (mode == READ){
           df = fopen(file,"r");
       } else {
           df = fopen(file,"w+");
       }
       if (df == 0)
           fprintf(stderr,"Can't open logfile %s, Error %s\n", file, strerror(errno));
   }
   if (df){
       if (setvbuf(df, NULL, _IOFBF , 1<<16) == 0){
           if (mode == READ){
               inputFile = df;
	   } else {
               outputFile = df;
	   }
       }
   }
   signal(SIGINT, sigintHandler);
   
   err = ftdi_stream(ftdi, callback, NULL, 8, 256);
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
   exit (0);
}

