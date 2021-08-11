"""
Dictionaries and constants used to 
"""

# Expected number of bytes for a given variable type
type_byte_lengths = {
    "char"		:	1,
    "uint8_t"	:	1,
    "int8_t"	:	1,
    "uint16_t"	:	2,
    "int16_t"	:	2,
    "uint32_t"	:	4,
    "int32_t"	:	4,
    "float"     :   4,
    "uint64_t"	:	8,
    "int64_t"	:	8,
    "double"    :   8
}

# Upper bound for a given variable type
type_range_positive = {
    "char"		:	255,
    "uint8_t"	:	(2**8)-1,
    "int8_t"	:	((2**8)/2)-1,
    "uint16_t"	:	(2**16)-1,
    "int16_t"	:	((2**16)/2)-1,
    "uint32_t"	:	(2**32)-1,
    "int32_t"	:	((2**32)/2)-1,
    "uint64_t"	:	(2**64)-1,
    "int64_t"	:	((2**64)/2)-1
}

# Lower bound for a given variable type
type_range_negative = {
    "char"		:	0,
    "uint8_t"	:	0,
    "int8_t"	:	-((2**8)/2),
    "uint16_t"	:	0,
    "int16_t"	:	-((2**16)/2),
    "uint32_t"	:	0,
    "int32_t"	:	-((2**32)/2),
    "uint64_t"	:	0,
    "int64_t"	:	-((2**64)/2)
}

# Used for packing/unpacking bytes
# See https://docs.python.org/3/library/struct.html for details on byte formatting
type_unpack_arg = {
    "char"		:	"\"<c\"",
    "uint8_t"	:	"\"<B\"",
    "int8_t"	:	"\"<b\"",
    "uint16_t"	:	"\"<H\"",
    "int16_t"	:	"\"<h\"",
    "uint32_t"	:	"\"<I\"",
    "int32_t"	:	"\"<i\"",
    "uint64_t"	:	"\"<Q\"",
    "int64_t"	:	"\"<q\"",
}