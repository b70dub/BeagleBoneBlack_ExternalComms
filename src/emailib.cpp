/*-----------------------------------------------------------------------------
 *
 * File: emailib.cpp
 * Author: Brian Ankeny
 * Platform: Beaglebone Black
 * Eclipse Luna Service Release 2 (4.4.2)
 * Modified From: example by Daniel Stenberg, modified to run on Beaglebone Black
 *                All Rights belong to their respective owners.
 *
 *-----------------------------------------------------------------------------

COPYRIGHT AND PERMISSION NOTICE

Copyright (c) 1996 - 2015, Daniel Stenberg, daniel@haxx.se.

All rights reserved.

Permission to use, copy, modify, and distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice
and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/





static const char *payload_text[] = {
  //"Date: Mon, 29 Nov 2010 21:57:29 +1100\r\n",
  "To: " TO "\r\n",
  "From: " FROM "(Controller Alert System)\r\n",
  "Cc: " CC "\r\n",
  //"Message-ID: <dcd7cb36-11db-487a-9f3a-e652a9458efd@rfcpedant.example.org>\r\n",
 // "Message-ID: <dcd7cb36-11db-487a-9f3a-e652a9458efd@rfcpedant.example.org>\r\n",
  "Subject: Test Alert Message\r\n",
  "\r\n", /* empty line to divide headers from body, see RFC5322 */
  "Issue: Deviation detected!!\r\n",
  "Machine: 7\r\n",
  "Cycle: 8\r\n",
  "Product: 110\r\n",
  "Baskets in machine: 6\r\n",
  "\r\n",
  "\r\n",
  "**This message was sent to you by a controller as a test. :)\r\n",
  "\r\n",
  "\r\n",
  NULL
};





#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <curl/curl.h>

#define MAXNUMSTRINGS 20
//#define MAXSTRINGLENGTH 512

namespace emailtools {

emailib::emailib() {
	// TODO Auto-generated constructor stub

}

emailib::~emailib() {
	// TODO Auto-generated destructor stub
}



struct upload_status {
  int lines_read;
};

//_________________________________________
// ::: Configuration and initialization :::


static size_t payload_source(void *ptr, size_t size, size_t nmemb, void *userp)
{
  struct upload_status *upload_ctx = (struct upload_status *)userp;
  const char *data;

  if((size == 0) || (nmemb == 0) || ((size*nmemb) < 1)) {
    return 0;
  }

  data = payload_text[upload_ctx->lines_read];

  if(data) {
    size_t len = strlen(data);
    memcpy(ptr, data, len);
    upload_ctx->lines_read++;

    return len;
  }

  return 0;
}

int emailib::CreateDeviationText(const char *ToField,const char *FromField, const char *CCField, int iRetortNum, int iCycleNum, int iProductNum, int iBasketNum)
{
	FILE* fp = NULL;

	int icount, iToLength, iFromLength, iCCLength, iMaxFieldLen, iCurrentLine;
	int iMaxLiteralLen = 50,							//size of longest string literal to be added to email add fields
		iMaxBuffSize = 5;
	char *Contents[MAXNUMSTRINGS];
	char RetortNumBuff[iMaxBuffSize];
	char CycleNumBuff[iMaxBuffSize];
	char ProductNumBuff[iMaxBuffSize];
	char BasketNumBuff[iMaxBuffSize];
	char *MessageHeader = "Deviation detected!!\r\n";
	char *BlankLine = "\r\n";
	char *MessageClose = "**This message was sent to you by a controller as a test. \r\n";
	char *LegalNotice = "IMPORTANT NOTICE: This e-mail, including any attachments, is solely for use by the addressee(s) named above. It may contain confidential information for the sole use of the intended recipient(s). Any review, use, distribution or disclosure by others is strictly prohibited.  If you are not the intended recipient (or authorized to receive for the recipient), please contact the sender by return e-mail and permanently destroy all copies of the original message.\r\n";


	//=================================================================================
	//Load the Email transport information
	//=================================================================================

	//Find the max required string length for the "To" field
	iToLength = strlen(ToField);

	if ((Contents[iCurrentLine] = malloc(sizeof(char) * (iToLength + iMaxLiteralLen))) == NULL)
	{
		printf("unable to allocate memory \n");
		return -1;
	}

	//Load the "To" field
	strcpy(Contents[iCurrentLine], "To: ");
	strcat(Contents[iCurrentLine], ToField);					//load the addresses
	strcpy(Contents[iCurrentLine], "\r\n");
	iCurrentLine++;

	//Find the max required string length for the "From" field
	iFromLength = strlen(FromField);

	if ((Contents[iCurrentLine] = malloc(sizeof(char) * (iFromLength + iMaxLiteralLen))) == NULL)
	{
		printf("unable to allocate memory \n");
		return -1;
	}
	//Load the "From" field
	strcpy(Contents[iCurrentLine], "From: ");
	strcat(Contents[iCurrentLine], FromField);					//load the addresses
	strcpy(Contents[iCurrentLine], "(Controller Alert System)\r\n");
	iCurrentLine++;


	//Find the max required string length for the "From" field
	iCCLength = strlen(CCField);

	if ((Contents[iCurrentLine] = malloc(sizeof(char) * (iCCLength + iMaxLiteralLen))) == NULL)
	{
		printf("unable to allocate memory \n");
		return -1;
	}

	//Load the "Cc" field
	strcpy(Contents[iCurrentLine], "Cc: ");
	strcat(Contents[iCurrentLine], CCField);					//load the addresses
	strcpy(Contents[iCurrentLine], "\r\n");
	iCurrentLine++;

	//=================================================================================
	//Load the Email payload data
	//=================================================================================

	//Allocate space for message header and load it
	if ((Contents[iCurrentLine] = malloc(sizeof(char) * (strlen(MessageHeader)))) == NULL)
	{
		printf("unable to allocate memory \n");
		return -1;
	}
	strcat(Contents[iCurrentLine], MessageHeader);
	iCurrentLine++;



	//Allocate space for the process information (Retort Number, Cycle Number, Product Number, Number of Baskets)
	for(int i = iCurrentLine; i <= (iCurrentLine+4); i++)
	{
		if ((Contents[i] = malloc(sizeof(char) * (iMaxBuffSize + 25))) == NULL)
		{
			printf("unable to allocate memory \n");
			return -1;
		}
	}

	//Load the Machine Number
	strcpy(Contents[iCurrentLine], "Retort: ");
	snprintf(RetortNumBuff, iMaxBuffSize, "%d", iRetortNum);
	strcat(Contents[iCurrentLine], RetortNumBuff);
	strcpy(Contents[iCurrentLine], "\r\n");
	iCurrentLine++;


	//Load the Cycle Number
	strcpy(Contents[iCurrentLine], "Cycle: ");
	snprintf(CycleNumBuff, iMaxBuffSize, "%d", iCycleNum);
	strcat(Contents[iCurrentLine], CycleNumBuff);
	strcpy(Contents[iCurrentLine], "\r\n");
	iCurrentLine++;

	//Load Product Number
	strcpy(Contents[iCurrentLine], "Product: ");
	snprintf(ProductNumBuff, iMaxBuffSize, "%d", iProductNum);
	strcat(Contents[iCurrentLine], ProductNumBuff);
	strcpy(Contents[iCurrentLine], "\r\n");
	iCurrentLine++;

	//Load Basket Number
	strcpy(Contents[iCurrentLine], "Baskets in Machine: ");
	snprintf(BasketNumBuff, iMaxBuffSize, "%d", iBasketNum);
	strcat(Contents[iCurrentLine], BasketNumBuff);
	strcpy(Contents[iCurrentLine], "\r\n");
	iCurrentLine++;


	//Allocate space for blank lines
	for(i = iCurrentLine; i <= (iCurrentLine+1); i++)
	{
		if ((Contents[i] = malloc(sizeof(char) * (strlen(BlankLine)))) == NULL)
		{
			printf("unable to allocate memory \n");
			return -1;
		}
	}
	strcat(Contents[iCurrentLine], BlankLine);
	iCurrentLine++;
	strcat(Contents[iCurrentLine], BlankLine);
	iCurrentLine++;

	//Allocate space for closing message
	if ((Contents[iCurrentLine] = malloc(sizeof(char) * (strlen(MessageClose)))) == NULL)
	{
		printf("unable to allocate memory \n");
		return -1;
	}
	strcat(Contents[iCurrentLine], MessageClose);
	iCurrentLine++;

	//Allocate space for blank lines
	for(i = iCurrentLine; i <= (iCurrentLine+1); i++)
	{
		if ((Contents[i] = malloc(sizeof(char) * (strlen(BlankLine)))) == NULL)
		{
			printf("unable to allocate memory \n");
			return -1;
		}
	}
	strcat(Contents[iCurrentLine], BlankLine);
	iCurrentLine++;
	strcat(Contents[iCurrentLine], BlankLine);
	iCurrentLine++;

	//Allocate space for legal notice
	if ((Contents[iCurrentLine] = malloc(sizeof(char) * (strlen(LegalNotice)))) == NULL)
	{
		printf("unable to allocate memory \n");
		return -1;
	}
	strcat(Contents[iCurrentLine], LegalNotice);
	iCurrentLine++;

	//Close the email
	strcpy(Contents[iCurrentLine], NULL);


	//=================================================================================
	//Now write email information to a file
	//=================================================================================


	if(NULL == (fp = fopen("DeviationEmail.txt","w")))
	{
		printf("\n fopen() Error!!!\n");
		return -1;
	}

	printf("\n File opened successfully\n");

	for(int iLineCount = 0; iLineCount <= iCurrentLine; iLineCount++)
	{
		if(strlen(Contents[iLineCount]) != fwrite(Contents[iLineCount], 1, strlen(Contents[iLineCount]), fp)
		{
			printf("\n ERROR: fwrite() failed\n");
			return -1;
		}
	}

	printf("\n fwrite() successful, data written to text file\n");

	if(fclose(fd))
	{
		printf("\n ERROR closing file!! \n");
		return -1;
	}

	//Free up the allocated memory before leaving!!
	for(int ifreecount = 0; ifreecount <= iCurrentLine; ifreecount++)
		free (Contents[ifreecount]);

	return 1;
#endif
}


int emailib::sendemail(const char *ToField,const char *FromField, const char *CCField)
{
	CURL *curl;
	  CURLcode res = CURLE_OK;
	  struct curl_slist *recipients = NULL;
	  struct upload_status upload_ctx;

	  upload_ctx.lines_read = 0;

	  curl = curl_easy_init();
	  if(curl) {
	    /* Set username and password */
	    curl_easy_setopt(curl, CURLOPT_USERNAME, "xxxxxx@gmail.com");
	    curl_easy_setopt(curl, CURLOPT_PASSWORD, "xxxxx");

	    /* This is the URL for your mailserver. Note the use of port 587 here,
	     * instead of the normal SMTP port (25). Port 587 is commonly used for
	     * secure mail submission (see RFC4403), but you should use whatever
	     * matches your server configuration. */
	    curl_easy_setopt(curl, CURLOPT_URL, "smtp://smtp.gmail.com:587");

	    /* In this example, we'll start with a plain text connection, and upgrade
	     * to Transport Layer Security (TLS) using the STARTTLS command. Be careful
	     * of using CURLUSESSL_TRY here, because if TLS upgrade fails, the transfer
	     * will continue anyway - see the security discussion in the libcurl
	     * tutorial for more details. */
	    curl_easy_setopt(curl, CURLOPT_USE_SSL, (long)CURLUSESSL_ALL);

	    /* If your server doesn't have a valid certificate, then you can disable
	     * part of the Transport Layer Security protection by setting the
	     * CURLOPT_SSL_VERIFYPEER and CURLOPT_SSL_VERIFYHOST options to 0 (false).
	     *   curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
	     *   curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
	     * That is, in general, a bad idea. It is still better than sending your
	     * authentication details in plain text though.
	     * Instead, you should get the issuer certificate (or the host certificate
	     * if the certificate is self-signed) and add it to the set of certificates
	     * that are known to libcurl using CURLOPT_CAINFO and/or CURLOPT_CAPATH. See
	     * docs/SSLCERTS for more information. */
	    //curl_easy_setopt(curl, CURLOPT_CAINFO, "/path/to/certificate.pem"); -- BTA commented out to prevent certificate errors

	    /* Note that this option isn't strictly required, omitting it will result in
	     * libcurl sending the MAIL FROM command with empty sender data. All
	     * autoresponses should have an empty reverse-path, and should be directed
	     * to the address in the reverse-path which triggered them. Otherwise, they
	     * could cause an endless loop. See RFC 5321 Section 4.5.5 for more details.
	     */
	    curl_easy_setopt(curl, CURLOPT_MAIL_FROM, FromField);

	    /* Add two recipients, in this particular case they correspond to the
	     * To: and Cc: addressees in the header, but they could be any kind of
	     * recipient. */
	    recipients = curl_slist_append(recipients, ToField);
	    recipients = curl_slist_append(recipients, CCField);
	    curl_easy_setopt(curl, CURLOPT_MAIL_RCPT, recipients);

	    /* We're using a callback function to specify the payload (the headers and
	     * body of the message). You could just use the CURLOPT_READDATA option to
	     * specify a FILE pointer to read from. */
	    curl_easy_setopt(curl, CURLOPT_READFUNCTION, payload_source);
	    curl_easy_setopt(curl, CURLOPT_READDATA, &upload_ctx);
	    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

	    /* Since the traffic will be encrypted, it is very useful to turn on debug
	     * information within libcurl to see what is happening during the transfer.
	     */
	    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

	    /* Send the message */
	    res = curl_easy_perform(curl);

	    /* Check for errors */
	    if(res != CURLE_OK)
	      fprintf(stderr, "curl_easy_perform() failed: %s\n",
	              curl_easy_strerror(res));

	    /* Free the list of recipients */
	    curl_slist_free_all(recipients);

	    /* Always cleanup */
	    curl_easy_cleanup(curl);
	  }

	  return (int)res;
#endif
}










} /* namespace serialcomms */
