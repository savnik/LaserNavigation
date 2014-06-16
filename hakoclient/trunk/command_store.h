#ifndef COMMAND_STORE_HDR
#define COMMAND_STORE_HDR

/* This "module" provides storage and filtering for embedded SMR-CL commands 
 * in route plan XML files. They will convert MATLAB string literal markers '''
 * with ordinary C language string literal markers '"'. This in order to allow
 * MATLAB conventions in XML files, but convert them to C notation (also used
 * by the SMR-CL */
/* The command strings are passed to this module which then stores them in
 * "persistent" storage and returns pointers to them. These pointers should
 * then be stored as waypoint information. */ 

void CS_reset(void);
/* Resets the command store - i.e. zeroes out the memory allocated for command
 * strings. Note that any pointers returned previously will no longer point
 * to valid strings! If the store is populated again (new route plan file) then
 * these pointers might again point to "strings" - but probably not at the right
 * start (or the right string).
 * It is not expected that it will be necessary to call this function! (Plenty
 * of storage allocated and storage valid at start-up). */

const char * CS_store(const char * xml_attr_cmd_str);
/* Store the string pointed at by 'xml_attr_cmd_str' in the command store and
 * return a pointer to this structure - note that it is "const char", thus the
 * compiler should warn if any "consumer" tries to corrupt the database.
 * The input string is expected to be in raw XML attribute format for embedded
 * commands (i.e. ''' used in place of '"' for string literal markers).
 * The string is then stored in the storage in command format (i.e. '"' are 
 * used as string literal markers) and a pointer to this is returned.
 * 
 * If passed a NULL pointer it will return a NULL pointer.
 * If some error occurs during the "storage" process a NULL pointer is returned
 * (i.e. the string is discarded). Might be changed to a pointer to a "stop"
 * command.
 * If the filtering and storage process proceeds without problems, then a pointer
 * to the storage is returned - and this pointer will be valid until 'CS_reset'
 * is called. */

/* Note: if one "store" operation has failed (for a long string) due to lack of
 * storage, the next "store" might very well succeed if the string is shorter! */

#endif /* COMMAND_STORE_HDR */
