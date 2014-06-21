
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <inttypes.h>
#include <string.h>

#include <libasmP.h>


int
asm_addsleb128 (asmscn, num)
     AsmScn_t *asmscn;
     int32_t num;
{
  if (asmscn == NULL)
    return -1;

  if (asmscn->type == SHT_NOBITS && unlikely (num != 0))
    {
      __libasm_seterrno (ASM_E_TYPE);
      return -1;
    }

  if (unlikely (asmscn->ctx->textp))
    printf ("\t.sleb128\t%" PRId32 "\n", num);
  else
    {
      char tmpbuf[(sizeof (num) * 8 + 6) / 7];
      char *dest = tmpbuf;
      uint32_t byte;
      int32_t endval = num >> 31;

      if (num == 0)
	byte = 0;
      else
	while (1)
	  {
	    byte = num & 0x7f;

	    num >>= 7;
	    if (num == endval)
	      /* This is the last byte.  */
	      break;

	    *dest++ = byte | 0x80;
	  }

      *dest++ = byte;

      /* Number of bytes produced.  */
      size_t nbytes = dest - tmpbuf;

      /* Make sure we have enough room.  */
      if (__libasm_ensure_section_space (asmscn, nbytes) != 0)
	return -1;

      /* Copy the bytes.  */
      if (likely (asmscn->type != SHT_NOBITS))
	memcpy (&asmscn->content->data[asmscn->content->len], tmpbuf, nbytes);

      /* Adjust the pointer in the data buffer.  */
      asmscn->content->len += nbytes;

      /* Increment the offset in the (sub)section.  */
      asmscn->offset += nbytes;
    }

  return 0;
}
