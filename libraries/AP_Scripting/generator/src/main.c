#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

char keyword_alias[]               = "alias";
char keyword_rename[]              = "rename";
char keyword_ap_object[]           = "ap_object";
char keyword_comment[]             = "--";
char keyword_depends[]             = "depends";
char keyword_enum[]                = "enum";
char keyword_field[]               = "field";
char keyword_include[]             = "include";
char keyword_method[]              = "method";
char keyword_operator[]            = "operator";
char keyword_read[]                = "read";
char keyword_scheduler_semaphore[] = "scheduler-semaphore";
char keyword_semaphore[]           = "semaphore";
char keyword_semaphore_pointer[]   = "semaphore-pointer";
char keyword_singleton[]           = "singleton";
char keyword_userdata[]            = "userdata";
char keyword_write[]               = "write";
char keyword_literal[]             = "literal";
char keyword_reference[]           = "reference";
char keyword_deprecate[]           = "deprecate";
char keyword_manual[]              = "manual";
char keyword_global[]              = "global";
char keyword_creation[]            = "creation";
char keyword_manual_operator[]     = "manual_operator";
char keyword_operator_getter[]     = "operator_getter";
char keyword_field_valid_mask[]    = "valid_mask";
char keyword_get[]                 = "get";


// attributes (should include the leading ' )
char keyword_attr_enum[]    = "'enum";
char keyword_attr_literal[] = "'literal";
char keyword_attr_null[]    = "'Null";
char keyword_attr_reference[]  = "'Ref";
char keyword_attr_array[]      = "'array";
char keyword_attr_no_range_check[] = "'skip_check";

// type keywords
char keyword_boolean[]  = "boolean";
char keyword_float[]    = "float";
char keyword_int8_t[]   = "int8_t";
char keyword_int16_t[]  = "int16_t";
char keyword_int32_t[]  = "int32_t";
char keyword_string[]   = "string";
char keyword_uint8_t[]  = "uint8_t";
char keyword_uint16_t[] = "uint16_t";
char keyword_uint32_t[] = "uint32_t";
char keyword_void[]     = "void";

enum error_codes {
  ERROR_OUT_OF_MEMORY   = 1, // ran out of memory
  ERROR_HEADER          = 2, // header keyword not followed by a header to include
  ERROR_UNKNOWN_KEYWORD = 3, // a keyword we didn't know how to handle
  ERROR_USERDATA        = 4, // userdata
  ERROR_INTERNAL        = 5, // internal error of some form
  ERROR_GENERAL         = 6, // general error
  ERROR_SINGLETON       = 7, // singletons
  ERROR_DEPENDS         = 8, // dependencies
  ERROR_DOCS            = 9, // Documentation
  ERROR_GLOBALS         = 10,
};

struct header {
  struct header *next;
  char *name; // name of the header to include (not sanitized)
  int line; // line of the file declared on
  char *dependency;
};

struct generator_state {
  char line[1<<14];
  int line_num; // current line read in
  int token_num; // current token on the current line
  char *token;
};

FILE *description;
FILE *header;
FILE *source;
FILE *docs;

static struct generator_state state;
static struct header * headers;

enum trace_level {
  TRACE_TOKENS    = (1 << 0),
  TRACE_HEADER    = (1 << 1),
  TRACE_GENERAL   = (1 << 2),
  TRACE_USERDATA  = (1 << 3),
  TRACE_SINGLETON = (1 << 4),
  TRACE_DEPENDS   = (1 << 5),
};

enum access_flags {
  ACCESS_FLAG_READ  = (1 << 0),
  ACCESS_FLAG_WRITE = (1 << 1),
};

enum field_type {
  TYPE_BOOLEAN = 0,
  TYPE_FLOAT,
  TYPE_INT8_T,
  TYPE_INT16_T,
  TYPE_INT32_T,
  TYPE_UINT8_T,
  TYPE_UINT16_T,
  TYPE_UINT32_T,
  TYPE_NONE,
  TYPE_STRING,
  TYPE_ENUM,
  TYPE_LITERAL,
  TYPE_USERDATA,
  TYPE_AP_OBJECT
};

const char * type_labels[TYPE_USERDATA + 1] = { "bool",
                                                "float",
                                                "int8_t",
                                                "int16_t",
                                                "int32_t",
                                                "uint8_t",
                                                "uint16_t",
                                                "void",
                                                "string",
                                                "enum",
                                                "userdata",
                                                "ap_object",
                                              };

enum operator_type {
  OP_ADD  = (1U << 0),
  OP_SUB  = (1U << 1),
  OP_MUL  = (1U << 2),
  OP_DIV  = (1U << 3),
  OP_MOD  = (1U << 4),
  OP_BAND = (1U << 5),
  OP_BOR  = (1U << 6),
  OP_BXOR = (1U << 7),
  OP_SHL  = (1U << 8),
  OP_SHR  = (1U << 9),
  OP_EQ   = (1U << 10),
  OP_LT   = (1U << 11),
  OP_LE   = (1U << 12),
  OP_BNOT = (1U << 13),
  OP_MANUAL = (1U << 14),
  OP_LAST
};

enum access_type {
  ACCESS_VALUE = 0,
  ACCESS_REFERENCE,
};

struct range_check {
  // store the requested range check as a string
  // we will check that it's a numeric form of some type, but keep it as a string rather then a casted version
  char *low;
  char *high;
};

enum type_flags {
  TYPE_FLAGS_NULLABLE = (1U << 1),
  TYPE_FLAGS_ENUM     = (1U << 2),
  TYPE_FLAGS_REFERENCE = (1U << 3),
  TYPE_FLAGS_NO_RANGE_CHECK = (1U << 4),
};

struct type {
  struct range_check *range;
  enum field_type type;
  enum access_type access;
  uint32_t flags;
  union {
    struct ud {
      char *name;
      char *sanitized_name;
    } ud;
    char *enum_name;
    char *literal;
  } data;
  struct {
    char *name;
    char *value;
  } valid_mask;
};

int TRACE_LEVEL = 0;

void trace(const int trace, const char *message, ...) {
  if (trace & TRACE_LEVEL) {
    char * fmt = malloc(strlen(message)+1024);
    if (fmt == NULL) {
      exit(ERROR_OUT_OF_MEMORY);
    }
  
    sprintf(fmt, "TRACE: %s\n", message);
  
    va_list args;
    va_start(args, message);
    vfprintf(stderr, fmt, args);
    va_end(args);
    free(fmt);
    fmt = NULL;
  }
}

void error(const int code, const char *message, ...) __attribute__ ((noreturn));
void error(const int code, const char *message, ...) {
  char * fmt = malloc(strlen(message)+1024);
  if (fmt == NULL) {
    exit(ERROR_OUT_OF_MEMORY);
  }

  if (state.line_num >= 0) {
    sprintf(fmt, "Error (line %d): %s\n", state.line_num, message);
  } else {
    sprintf(fmt, "Error: %s\n", message);
  }

  va_list args;
  va_start(args, message);
  vfprintf(stderr, fmt, args);
  va_end(args);
  free(fmt);
  fmt = NULL;
  exit(code);
}

char *token_delimiters = " \n";

char * next_token(void) {
  state.token = strtok(NULL, token_delimiters);
  state.token_num++;
  trace(TRACE_TOKENS, "Token %d:%d %s", state.line_num, state.token_num, state.token);
  if ((state.token!= NULL) && (strcmp(state.token, keyword_comment) == 0)) {
    trace(TRACE_TOKENS, "Detected comment %d", state.line_num);
    state.token = NULL; // burn the line
  }
  return state.token;
}

char * start_line(void) {
  while (fgets(state.line, sizeof(state.line)/sizeof(state.line[0]), description) != NULL) {//state.line = readline(NULL))) {
      state.line_num++;
    
      const size_t length = strlen(state.line);
      if (length > 1 && state.line[length - 2] == '\r') {
        trace(TRACE_TOKENS, "Discarding carriage return");
        if (length == 2) { // empty line of just carriage return, loop again
          continue;
        }
        state.line[length - 2] = '\0';
      } else if (length > 0 && state.line[length - 1] == '\n') {
        trace(TRACE_TOKENS, "Discarding newline");
        if (length == 1) { // empty line of just carriage return, loop again
          continue;
        }
        state.line[length - 1] = '\0';
      }

      state.token = strtok(state.line, token_delimiters);
      state.token_num = 1;
      trace(TRACE_TOKENS, "Start of line token %d:%d %s", state.line_num, state.token_num, state.token);

      if (state.token != NULL) {
        break;
      }
  }

  return state.token;
}

// thin wrapper for malloc that exits if we can't allocate memory, and memsets the allocated chunk
void *allocate(const size_t size) {
  void *data = malloc(size);
  if (data == NULL) {
    error(ERROR_OUT_OF_MEMORY, "Out of memory.");
  } else {
    memset(data, 0, size);
  }
  return data;
}

// lazy helper that allocates a storage buffer and does strcpy for us
void string_copy(char **dest, const char * src) {
  *dest = (char *)allocate(strlen(src) + 1);
  strcpy(*dest, src);
}

void handle_header(void) {
  trace(TRACE_HEADER, "Parsing a header");

  // find the new header
  char * name = next_token();
  if (name == NULL) {
    error(ERROR_HEADER, "Header must be followed by the name of the header to include");
  }

  // search for duplicates
  struct header *node = headers;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }
  if (node != NULL) {
    error(ERROR_HEADER, "Header %s was already included on line %d", name, node->line);
  }

  // add to the list of headers
  node = (struct header *)allocate(sizeof(struct header));
  node->next = headers;
  node->line = state.line_num;
  node->name = (char *)allocate(strlen(name) + 1);
  strcpy(node->name, name);

  // add depends
  char * key_word = next_token();
  char *depends = NULL;
  if (key_word != NULL) {
    if (strcmp(key_word, keyword_depends) == 0) {
        depends = strtok(NULL, "");
        if (depends == NULL) {
          error(ERROR_HEADER, "Expected a depends string for %s", name);
        }
        string_copy(&(node->dependency), depends);
    } else {
      error(ERROR_HEADER, "Received an unsupported keyword on a header: %s", key_word);
    }
  }

  headers = node;

  trace(TRACE_HEADER, "Added header %s", name);

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Header contained an unexpected extra token: %s", state.token);
  }
}

enum userdata_type {
  UD_USERDATA,
  UD_SINGLETON,
  UD_AP_OBJECT,
  UD_GLOBAL,
};

struct argument {
  struct argument * next;
  struct type type;
  int line_num; // line read from
  int token_num; // token number on the line
};

struct method {
  struct method * next;
  char *name;
  char *sanitized_name;  // sanitized name of the C++ singleton
  char *rename; // (optional) used for scripting access
  char *deprecate; // (optional) issue deprecation warning string on first call
  int line; // line declared on
  struct type return_type;
  struct argument * arguments;
  uint32_t flags; // filled out with TYPE_FLAGS
  char *dependency;
  int stats_id;
};

enum alias_type {
  ALIAS_TYPE_NONE,
  ALIAS_TYPE_MANUAL,
  ALIAS_TYPE_MANUAL_OPERATOR,
};

struct method_alias {
  struct method_alias *next;
  char *name;
  char *alias;
  int line;
  int num_args;
  int num_ret;
  enum alias_type type;
  char *dependency;
  int stats_id;
};

struct userdata_field {
  struct userdata_field * next;
  char * name;
  char * rename;
  struct type type; // field type, points to a string
  int line; // line declared on
  unsigned int access_flags;
  char * array_len; // literal array length
  char *dependency;
};

enum userdata_flags {
  UD_FLAG_SEMAPHORE         = (1U << 0),
  UD_FLAG_SCHEDULER_SEMAPHORE = (1U << 1),
  UD_FLAG_LITERAL = (1U << 2),
  UD_FLAG_SEMAPHORE_POINTER = (1U << 3),
  UD_FLAG_REFERENCE = (1U << 4),
};

struct userdata_enum {
  struct userdata_enum * next;
  char * name;     // enum name
};

struct userdata {
  struct userdata * next;
  char *name;  // name of the C++ singleton
  char *sanitized_name;  // sanitized name of the C++ singleton
  char *rename; // (optional) used for scripting access
  struct userdata_field *fields;
  struct method *methods;
  struct method_alias *method_aliases;
  struct userdata_enum *enums;
  enum userdata_type ud_type;
  uint32_t operations; // bitset of enum operation_types
  int flags; // flags from the userdata_flags enum
  char *dependency;
  char *creation; // name of a manual creation function if set, note that this will not be used internally
  int creation_args; // number of args for custom creation function
  char *operator_getter; // Custom function to get values for use in operators
  char *get_string; // custom string to get object pointer
};

static struct userdata *parsed_userdata;
static struct userdata *parsed_ap_objects;
static struct userdata *parsed_globals;

#define MAX_STATS_IDS 2048
static const char *stats_names[MAX_STATS_IDS];
static int next_stats_id = 0;
// set once the bindings were emitted: a second emission (remote mode) reuses their ids
static int stats_ids_frozen = 0;
static int assign_stats_id(const char *ud_name, const char *method_name) {
  if (next_stats_id >= MAX_STATS_IDS) {
    error(ERROR_GENERAL, "Too many bindings for stats ids");
  }
  char *n = (char *)allocate(strlen(ud_name) + strlen(method_name) + 2);
  sprintf(n, "%s:%s", ud_name, method_name);
  stats_names[next_stats_id] = n;
  return next_stats_id++;
}

void sanitize_character(char **str, char character) {
  char *position = strchr(*str, character);
  while (position) {
    *position = '_';
    position = strchr(position, character);
  }
}

void sanitize_name(char **dest, char *src) {
  *dest = (char *)allocate(strlen(src) + 1);
  strcpy(*dest, src);

  sanitize_character(dest, ':');
  sanitize_character(dest, '.');
  sanitize_character(dest, '<');
  sanitize_character(dest, '>');
  sanitize_character(dest, '(');
  sanitize_character(dest, ')');
  sanitize_character(dest, '-');

};


struct range_check *parse_range_check(enum field_type type) {
  char * low = next_token();
  if (low == NULL) {
    error(ERROR_USERDATA, "Missing low value for a range check (type: %s)", type_labels[type]);
  }

  trace(TRACE_TOKENS, "Range check: Low: %s", low);

  char * high  = next_token();
  if (high == NULL) {
    error(ERROR_USERDATA, "Missing high value for a range check");
  }

  trace(TRACE_TOKENS, "Range check: High: %s", high);

  struct range_check *check = allocate(sizeof(struct range_check));

  string_copy(&(check->low), low);
  string_copy(&(check->high), high);

  return check;
}

// parses one or more access flags, leaves the token on the first non access token
// throws an error if no flags were found
unsigned int parse_access_flags(struct type * type) {
  unsigned int flags = 0;

  next_token();

  while(state.token != NULL) {
    trace(TRACE_TOKENS, "Possible access: %s", state.token);
    if (strcmp(state.token, keyword_read) == 0) {
      flags |= ACCESS_FLAG_READ;
    } else if (strcmp(state.token, keyword_write) == 0) {
      flags |= ACCESS_FLAG_WRITE;
      if ((type->flags & TYPE_FLAGS_NO_RANGE_CHECK) == 0) {
        switch (type->type) {
          case TYPE_FLOAT:
          case TYPE_INT8_T:
          case TYPE_INT16_T:
          case TYPE_INT32_T:
          case TYPE_UINT8_T:
          case TYPE_UINT16_T:
          case TYPE_UINT32_T:
          case TYPE_ENUM:
            type->range = parse_range_check(type->type);
            break;
          case TYPE_AP_OBJECT:
          case TYPE_USERDATA:
          case TYPE_BOOLEAN:
          case TYPE_STRING:
          case TYPE_LITERAL:
            // a range check is illogical
            break;
          case TYPE_NONE:
            error(ERROR_INTERNAL, "Can't access a NONE type");
        }
      }
    } else if (strcmp(state.token, keyword_field_valid_mask) == 0) {
      char * name = next_token();
      string_copy(&(type->valid_mask.name), name);
      char * value = next_token();
      string_copy(&(type->valid_mask.value), value);
    } else {
      error(ERROR_UNKNOWN_KEYWORD, "Unknown access provided: %s", state.token);
      break;
    }
    next_token();
  }

  trace(TRACE_TOKENS, "Parsed access flags: 0x%x", flags);

  if (flags == 0) {
    error(ERROR_USERDATA, "Expected to find an access specifier");
  }

  return flags;
}

#define TRUE  1
#define FALSE 0

enum type_restriction {
  TYPE_RESTRICTION_NONE         = 0,
  TYPE_RESTRICTION_OPTIONAL     = (1U << 1),
  TYPE_RESTRICTION_NOT_NULLABLE = (1U << 2),
};

enum range_check_type {
  RANGE_CHECK_NONE,
  RANGE_CHECK_MANDATORY,
};

int parse_type(struct type *type, const uint32_t restrictions, enum range_check_type range_type) {
  char *data_type = next_token();

  if (data_type == NULL) {
    if (restrictions & TYPE_RESTRICTION_OPTIONAL) {
      return FALSE;
    } else {
      error(ERROR_USERDATA, "Data type must be specified");
    }
  }

  if (data_type[0] == '&') {
    type->access = ACCESS_REFERENCE;
    data_type++; // drop the reference character
  } else {
    type->access = ACCESS_VALUE;
  }

  char *attribute = strchr(data_type, '\'');
  if (attribute != NULL) {
    if (strcmp(attribute, keyword_attr_enum) == 0) {
      type->flags |= TYPE_FLAGS_ENUM;
    } else if (strcmp(attribute, keyword_attr_literal) == 0) {
      type->type = TYPE_LITERAL;
    } else if (strcmp(attribute, keyword_attr_null) == 0) {
      if (restrictions & TYPE_RESTRICTION_NOT_NULLABLE) {
        error(ERROR_USERDATA, "%s is not nullable in this context", data_type);
      }
      type->flags |= TYPE_FLAGS_NULLABLE;
    } else if (strcmp(attribute, keyword_attr_reference) == 0) {
      type->flags |= TYPE_FLAGS_REFERENCE;
    } else if (strcmp(attribute, keyword_attr_no_range_check) == 0) {
      type->flags |= TYPE_FLAGS_NO_RANGE_CHECK;
    } else {
      error(ERROR_USERDATA, "Unknown attribute: %s", attribute);
    }
    attribute[0] = 0;
  }

  if ((type->access == ACCESS_REFERENCE) && ((type->flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) == 0)) {
      error(ERROR_USERDATA, "Only support references access with 'Null or 'Ref keyword");
  }

  if (strcmp(data_type, keyword_boolean) == 0) {
    type->type = TYPE_BOOLEAN;
  } else if (strcmp(data_type, keyword_float) == 0) {
    type->type = TYPE_FLOAT;
  } else if (strcmp(data_type, keyword_int8_t) == 0) {
    type->type = TYPE_INT8_T;
  } else if (strcmp(data_type, keyword_int16_t) == 0) {
    type->type = TYPE_INT16_T;
  } else if (strcmp(data_type, keyword_int32_t) == 0) {
    type->type = TYPE_INT32_T;
  } else if (strcmp(data_type, keyword_uint8_t) == 0) {
    type->type = TYPE_UINT8_T;
  } else if (strcmp(data_type, keyword_uint16_t) == 0) {
    type->type = TYPE_UINT16_T;
  } else if (strcmp(data_type, keyword_uint32_t) == 0) {
    type->type = TYPE_UINT32_T;
  } else if (strcmp(data_type, keyword_string) == 0) {
    type->type = TYPE_STRING;
  } else if (strcmp(data_type, keyword_void) == 0) {
    type->type = TYPE_NONE;
  } else if (type->flags & TYPE_FLAGS_ENUM) {
    type->type = TYPE_ENUM;
    string_copy(&(type->data.enum_name), data_type);
  } else if (type->type == TYPE_LITERAL) {
    string_copy(&(type->data.literal), data_type);
  } else if (strcmp(data_type, keyword_rename) == 0) {
    error(ERROR_USERDATA, "Cant add rename to unknown function");
  } else {
    // this must be a user data or an ap_object, check if it's already been declared as an object
    struct userdata *node = parsed_ap_objects;
    while (node != NULL && strcmp(node->name, data_type)) {
      node = node->next;
    }
    if (node != NULL) {
      type->type = TYPE_AP_OBJECT;
    } else {
      // assume that this is a user data, we can't validate this until later though
      type->type = TYPE_USERDATA;
    }
    string_copy(&(type->data.ud.name), data_type);
    sanitize_name(&(type->data.ud.sanitized_name), type->data.ud.name);
  }

  // only allow no range check on float, int32 and uint32
  if (type->flags & TYPE_FLAGS_NO_RANGE_CHECK) {
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
        break;
      default:
        error(ERROR_USERDATA, "%s types cannot skip range check", data_type);
        break;
    }
  }

  // sanity check that only supported types are nullable
  if (type->flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) {
    // a switch is a very verbose way to do this, but forces users to consider new types added
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
      case TYPE_BOOLEAN:
      case TYPE_STRING:
      case TYPE_ENUM:
      case TYPE_USERDATA:
        break;
      case TYPE_AP_OBJECT:
      case TYPE_LITERAL:
      case TYPE_NONE:
        if (type->flags & TYPE_FLAGS_NULLABLE) {
          error(ERROR_USERDATA, "%s types cannot be nullable", data_type);
        } else {
          error(ERROR_USERDATA, "%s types cannot be passed as reference", data_type);
        }
        break;
    }
  }

  // add range checks, unless disabled or a nullable type
  if (range_type != RANGE_CHECK_NONE && !(type->flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE | TYPE_FLAGS_NO_RANGE_CHECK))) {
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
      case TYPE_ENUM:
        type->range = parse_range_check(type->type);
        break;
      case TYPE_AP_OBJECT:
      case TYPE_BOOLEAN:
      case TYPE_NONE:
      case TYPE_STRING:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        // no sane range checks, so we can ignore this
        break;
    }
  }
  return TRUE;
}

void handle_userdata_enum(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a userdata enum");

  char * enum_name;
  while ((enum_name = next_token()) != NULL) {
    trace(TRACE_USERDATA, "Adding enum %s", enum_name);
    struct userdata_enum *ud_enum = (struct userdata_enum *) allocate(sizeof(struct userdata_enum));
    ud_enum->next = data->enums;
    string_copy(&(ud_enum->name), enum_name);
    data->enums = ud_enum;
  }
}

void handle_userdata_field(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a userdata field");

  // find the field name
  char * token = next_token();
  if (token == NULL) {
    error(ERROR_USERDATA, "Missing a field name for userdata %s", data->name);
  }

  size_t split = strcspn(token, "\'");
  char * field_name;
  if (split == strlen(token)) {
    string_copy(&field_name, token);
  } else {
    field_name = (char *)allocate(split+1);
    memcpy(field_name, token, split);
  }

  struct userdata_field * field = data->fields;
  while (field != NULL && strcmp(field->name, field_name)) {
    field = field-> next;
  }
  if (field != NULL) {
    char *token = next_token();
    if (strcmp(token, keyword_rename) == 0) {
      char *rename = next_token();
      string_copy(&(field->rename), rename);

    } else if (strcmp(token, keyword_depends) == 0) {
      char *dependency = strtok(NULL, "");
      if (dependency == NULL) {
        error(ERROR_USERDATA, "Expected dependency string for %s %s on line", data->name, field_name, state.line_num);
      }
      string_copy(&(field->dependency), dependency);

    } else {
      error(ERROR_USERDATA, "Field %s already exists in userdata %s (declared on %d)", field_name, data->name, field->line);
    }
    return;
  }

  trace(TRACE_USERDATA, "Adding field %s", field_name);
  field = (struct userdata_field *)allocate(sizeof(struct userdata_field));
  field->next = data->fields;
  data->fields = field;
  field->line = state.line_num;
  string_copy(&(field->name), field_name);

  char *attribute = strchr(token, '\'');
  if (attribute != NULL) {
    if (strcmp(attribute, keyword_attr_array) != 0) {
      error(ERROR_USERDATA, "Unknown field attribute %s for userdata %s field %s", attribute, data->name, field_name);
    }
    char * token = next_token();
    string_copy(&(field->array_len), token);
    trace(TRACE_USERDATA, "userdata %s field %s array length %s", data->name, field->name, field->array_len);
  }

  parse_type(&(field->type), TYPE_RESTRICTION_NOT_NULLABLE, RANGE_CHECK_NONE);
  field->access_flags = parse_access_flags(&(field->type));
}

void handle_method(struct userdata *node) {
  trace(TRACE_USERDATA, "Adding a method");
  char * parent_name = node->name;

  // find the field name
  char * name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Missing method name for %s", parent_name);
  }
  
  struct method * method = node->methods;
  while (method != NULL && strcmp(method->name, name)) {
    method = method-> next;
  }
  if (method != NULL) {
    char *token = next_token();
    if (strcmp(token, keyword_rename) == 0) {
      char *rename = next_token();
      string_copy(&(method->rename), rename);
      return;

    } else if (strcmp(token, keyword_alias) == 0) {
      char *alias_name = next_token();

      struct method * method = node->methods;
      while (method != NULL && strcmp(method->name, alias_name)) {
        method = method-> next;
      }
      if (method != NULL) {
        error(ERROR_USERDATA, "Method %s already exists for %s (declared on %d) cannot alias to %s", alias_name, parent_name, method->line, name);
      }


      struct method_alias *alias = allocate(sizeof(struct method_alias));
      string_copy(&(alias->name), name);
      string_copy(&(alias->alias), alias_name);
      alias->line = state.line_num;
      alias->next = node->method_aliases;
      node->method_aliases = alias;
      return;

    } else if (strcmp(token, keyword_deprecate) == 0) {
      char *deprecate = strtok(NULL, "");
      if (deprecate == NULL) {
        error(ERROR_USERDATA, "Expected a deprecate string for %s %s on line", parent_name, name, state.line_num);
      }
      string_copy(&(method->deprecate), deprecate);
      return;

    } else if (strcmp(token, keyword_depends) == 0) {
      char *dependency = strtok(NULL, "");
      if (dependency == NULL) {
        error(ERROR_USERDATA, "Expected dependency string for %s %s on line", parent_name, name, state.line_num);
      }
      string_copy(&(method->dependency), dependency);
      return;

    }
    error(ERROR_USERDATA, "Method %s already exists for %s (declared on %d)", name, parent_name, method->line);
  }

  struct method_alias * alias = node->method_aliases;
  while (alias != NULL && strcmp(alias->alias, name)) {
    alias = alias-> next;
  }
  if (alias != NULL) {
    error(ERROR_USERDATA, "alias %s already exists for %s (declared on %d)", name, alias->name, alias->line);
  }

  trace(TRACE_USERDATA, "Adding method %s", name);
  method = allocate(sizeof(struct method));
  method->next = node->methods;
  node->methods = method;
  string_copy(&(method->name), name);
  sanitize_name(&(method->sanitized_name), name);
  method->line = state.line_num;

  parse_type(&(method->return_type), TYPE_RESTRICTION_NONE, RANGE_CHECK_NONE);

  // iterate the arguments
  struct type arg_type = {};
  while (parse_type(&arg_type, TYPE_RESTRICTION_OPTIONAL, RANGE_CHECK_MANDATORY)) {
    if (arg_type.type == TYPE_NONE) {
      error(ERROR_USERDATA, "Can't pass an empty argument to a method");
    }
    if ((method->return_type.type != TYPE_BOOLEAN) && (arg_type.flags & TYPE_FLAGS_NULLABLE)) {
      error(ERROR_USERDATA, "Nullable arguments are only available on a boolean method");
    }
    if ((method->return_type.type == TYPE_BOOLEAN) && (arg_type.flags & TYPE_FLAGS_REFERENCE)) {
      error(ERROR_USERDATA, "Use Nullable arguments on a boolean method, not 'Ref");
    }
    if (arg_type.flags & TYPE_FLAGS_NULLABLE) {
      method->flags |= TYPE_FLAGS_NULLABLE;
    }
    if (arg_type.flags & TYPE_FLAGS_REFERENCE) {
      method->flags |= TYPE_FLAGS_REFERENCE;
    }
    struct argument * arg = allocate(sizeof(struct argument));
    memcpy(&(arg->type), &arg_type, sizeof(struct type));
    arg->line_num = state.line_num;
    arg->token_num = state.token_num;
    if (method->arguments == NULL) {
      method->arguments = arg;
    } else {
      struct argument *tail = method->arguments;
      while (tail->next != NULL) {
        tail = tail->next;
      }
      tail->next = arg;
    }

    // reset the stack arg_type
    memset(&arg_type, 0, sizeof(struct type));
  }
}

void handle_manual(struct userdata *node, enum alias_type type) {
  char *name = next_token();
  if (name == NULL) {
    error(ERROR_SINGLETON, "Expected a lua name for manual %s method",node->name);
  }
  char *cpp_function_name = next_token();
  if (cpp_function_name == NULL) {
    error(ERROR_SINGLETON, "Expected a cpp name for manual method %s %s", node->name, name);
  }
  struct method_alias *alias = allocate(sizeof(struct method_alias));
  string_copy(&(alias->name), cpp_function_name);
  string_copy(&(alias->alias), name);
  alias->line = state.line_num;
  alias->type = type;

  if (type != ALIAS_TYPE_MANUAL_OPERATOR) {
    char *num_args = next_token();
    if (num_args == NULL) {
      error(ERROR_SINGLETON, "Expected number of args for manual method %s %s", node->name, name);
    }
    alias->num_args = atoi(num_args);

    char *num_ret = next_token();
    if (num_ret == NULL) {
      error(ERROR_SINGLETON, "Expected number of returns for manual method %s %s", node->name, name);
    }
    alias->num_ret = atoi(num_ret);
  }

  char *depends_keyword = next_token();
  if (depends_keyword != NULL) {
    if (strcmp(depends_keyword, keyword_depends) != 0) {
      error(ERROR_SINGLETON, "Expected depends keyword for manual method %s %s, got: %s", node->name, name, depends_keyword);
    } else {
      char *dependency = strtok(NULL, "");
      if (dependency == NULL) {
        error(ERROR_USERDATA, "Expected dependency string for global %s on line", name, state.line_num);
      }
      string_copy(&(alias->dependency), dependency);
    }
  }

  alias->next = node->method_aliases;
  node->method_aliases = alias;
}

void handle_operator(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a operator");

  if (data->ud_type != UD_USERDATA) {
    error(ERROR_USERDATA, "Operators are only allowed on userdata objects");
  }

  char *operator = next_token();
  if (operator == NULL) {
    error(ERROR_USERDATA, "Needed a symbol for the operator");
  }

  enum operator_type operation = OP_ADD;
  if (strcmp(operator, "+") == 0) {
    operation = OP_ADD;
  } else if (strcmp(operator, "-") == 0) {
    operation = OP_SUB;
  } else if (strcmp(operator, "*") == 0) {
    operation = OP_MUL;
  } else if (strcmp(operator, "/") == 0) {
    operation = OP_DIV;
  } else if (strcmp(operator, "%") == 0) {
    operation = OP_MOD;
  } else if (strcmp(operator, "&") == 0) {
    operation = OP_BAND;
  } else if (strcmp(operator, "|") == 0) {
    operation = OP_BOR;
  } else if (strcmp(operator, "^") == 0) {
    operation = OP_BXOR;
  } else if (strcmp(operator, "<<") == 0) {
    operation = OP_SHL;
  } else if (strcmp(operator, ">>") == 0) {
    operation = OP_SHR;
  } else if (strcmp(operator, "==") == 0) {
    operation = OP_EQ;
  } else if (strcmp(operator, "<") == 0) {
    operation = OP_LT;
  } else if (strcmp(operator, "<=") == 0) {
    operation = OP_LE;
  } else if (strcmp(operator, "~") == 0) {
    operation = OP_BNOT;
  } else {
    error(ERROR_USERDATA, "Unknown operation type: %s", operator);
  }

  if ((data->operations) & operation) {
    error(ERROR_USERDATA, "Operation %s was already defined for %s", operator, data->name);
  }

  trace(TRACE_USERDATA, "Adding operation %d to %s", operation, data->name);

  data->operations |= operation;

  if (next_token() != NULL) {
    error(ERROR_USERDATA, "Extra token on operation %s", operator);
  }
}

void handle_userdata(void) {
  trace(TRACE_USERDATA, "Adding a userdata");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the userdata");
  }

  struct userdata *node = parsed_userdata;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }
  if (node == NULL) {
    trace(TRACE_USERDATA, "Allocating new userdata for %s", name);
    node = (struct userdata *)allocate(sizeof(struct userdata));
    node->ud_type = UD_USERDATA;
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    sanitize_name(&(node->sanitized_name), node->name);
    node->next = parsed_userdata;
    parsed_userdata = node;
  } else {
    trace(TRACE_USERDATA, "Found existing userdata for %s", name);
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_USERDATA, "Expected a access type for userdata %s", name);
  }

  // match type
  if (strcmp(type, keyword_field) == 0) {
    handle_userdata_field(node);
  } else if (strcmp(type, keyword_operator) == 0) {
    handle_operator(node);
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(node);
  } else if (strcmp(type, keyword_enum) == 0) {
    handle_userdata_enum(node);
  } else if (strcmp(type, keyword_rename) == 0) {
    const char *rename = next_token();
    if (rename == NULL) {
      error(ERROR_SINGLETON, "Missing the name of the rename for userdata %s", node->name);
    }
    node->rename = (char *)allocate(strlen(rename) + 1);
    strcpy(node->rename, rename);

  } else if (strcmp(type, keyword_depends) == 0) {
      if (node->dependency != NULL) {
        error(ERROR_SINGLETON, "Userdata only support a single depends");
      }
      char *depends = strtok(NULL, "");
      if (depends == NULL) {
        error(ERROR_DEPENDS, "Expected a depends string for %s",node->name);
      }
      string_copy(&(node->dependency), depends);

  } else if (strcmp(type, keyword_manual) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL);

  } else if (strcmp(type, keyword_creation) == 0) {
      if (node->creation != NULL) {
        error(ERROR_SINGLETON, "Userdata only support a single creation function");
      }
      char *creation = next_token();
      if (creation == NULL) {
        error(ERROR_USERDATA, "Expected a creation string for %s",node->name);
      }
      char *num_args = next_token();
      if (num_args == NULL) {
        error(ERROR_SINGLETON, "Expected number of args for creation method %s", node->name);
      }
      node->creation_args = atoi(num_args);

      string_copy(&(node->creation), creation);

  } else if (strcmp(type, keyword_manual_operator) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL_OPERATOR);
    node->operations |= OP_MANUAL;

  } else if (strcmp(type, keyword_operator_getter) == 0) {
      if (node->operator_getter != NULL) {
        error(ERROR_USERDATA, "Userdata only support a single getter string");
      }
      char *name = next_token();
      if (name == NULL) {
        error(ERROR_USERDATA, "Expected a getter string for %s",node->name);
      }
      string_copy(&(node->operator_getter), name);

  } else {
    error(ERROR_USERDATA, "Unknown or unsupported type for userdata: %s", type);
  }

}

struct userdata *parsed_singletons = NULL;

void handle_singleton(void) {
  trace(TRACE_SINGLETON, "Adding a singleton");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the singleton");
  }

  struct userdata *node = parsed_singletons;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }

  if (node == NULL) {
    trace(TRACE_SINGLETON, "Allocating new singleton for %s", name);
    node = (struct userdata *)allocate(sizeof(struct userdata));
    node->ud_type = UD_SINGLETON;
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    sanitize_name(&(node->sanitized_name), node->name);
    node->next = parsed_singletons;
    parsed_singletons = node;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_SINGLETON, "Expected a access type for userdata %s", name);
  }

  if (strcmp(type, keyword_rename) == 0) {
    if (node->rename != NULL) {
      error(ERROR_SINGLETON, "rename of %s was already declared for %s", node->rename, node->name);
    }
    const char *rename = next_token();
    if (rename == NULL) {
      error(ERROR_SINGLETON, "Missing the name of the rename for %s", node->name);
    }
    node->rename = (char *)allocate(strlen(rename) + 1);
    strcpy(node->rename, rename);

  } else if (strcmp(type, keyword_semaphore) == 0) {
    node->flags |= UD_FLAG_SEMAPHORE;
  } else if (strcmp(type, keyword_scheduler_semaphore) == 0) {
    node->flags |= UD_FLAG_SCHEDULER_SEMAPHORE;
  } else if (strcmp(type, keyword_semaphore_pointer) == 0) {
    node->flags |= UD_FLAG_SCHEDULER_SEMAPHORE;
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(node);
  } else if (strcmp(type, keyword_enum) == 0) {
    handle_userdata_enum(node);
  } else if (strcmp(type, keyword_depends) == 0) {
    if (node->dependency != NULL) {
      error(ERROR_SINGLETON, "Singletons only support a single depends");
    }
    char *depends = strtok(NULL, "");
    if (depends == NULL) {
      error(ERROR_DEPENDS, "Expected a depends string for %s",node->name);
    }
    string_copy(&(node->dependency), depends);
  } else if (strcmp(type, keyword_get) == 0) {
    if (node->get_string != NULL) {
      error(ERROR_SINGLETON, "Singletons only support a single get");
    }
    char *get = strtok(NULL, "");
    if (get == NULL) {
      error(ERROR_DEPENDS, "Expected a get string for %s",node->name);
    }
    string_copy(&(node->get_string), get);
  } else if (strcmp(type, keyword_literal) == 0) {
    node->flags |= UD_FLAG_LITERAL;
  } else if (strcmp(type, keyword_field) == 0) {
    handle_userdata_field(node);
  } else if (strcmp(type, keyword_reference) == 0) {
    node->flags |= UD_FLAG_REFERENCE;
  } else if (strcmp(type, keyword_manual) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL);
  } else {
    error(ERROR_SINGLETON, "Singletons only support renames, methods, semaphore, depends, get, literal or manual keywords (got %s)", type);
  }

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Singleton contained an unexpected extra token: %s", state.token);
  }
}

void handle_ap_object(void) {
  trace(TRACE_SINGLETON, "Adding a ap_object");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the ap_object");
  }

  struct userdata *node = parsed_ap_objects;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }

  if (node == NULL) {
    trace(TRACE_USERDATA, "Allocating new ap_object for %s", name);
    node = (struct userdata *)allocate(sizeof(struct userdata));
    node->ud_type = UD_AP_OBJECT;
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    sanitize_name(&(node->sanitized_name), node->name);
    node->next = parsed_ap_objects;
    parsed_ap_objects = node;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_SINGLETON, "Expected a access type for ap_object %s", name);
  }

  if (strcmp(type, keyword_rename) == 0) {
    if (node->rename != NULL) {
      error(ERROR_SINGLETON, "Rename of %s was already declared for %s", node->rename, node->name);
    }
    const char *rename = next_token();
    if (rename == NULL) {
      error(ERROR_SINGLETON, "Missing the name of the rename for %s", node->name);
    }
    node->rename = (char *)allocate(strlen(rename) + 1);
    strcpy(node->rename, rename);

  } else if (strcmp(type, keyword_semaphore) == 0) {
    node->flags |= UD_FLAG_SEMAPHORE;
  } else if (strcmp(type, keyword_scheduler_semaphore) == 0) {
    node->flags |= UD_FLAG_SCHEDULER_SEMAPHORE;
  } else if (strcmp(type, keyword_semaphore_pointer) == 0) {
    node->flags |= UD_FLAG_SEMAPHORE_POINTER;
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(node);
  } else if (strcmp(type, keyword_depends) == 0) {
      if (node->dependency != NULL) {
        error(ERROR_SINGLETON, "AP_Objects only support a single depends");
      }
      char *depends = strtok(NULL, "");
      if (depends == NULL) {
        error(ERROR_DEPENDS, "Expected a depends string for %s",node->name);
      }
      string_copy(&(node->dependency), depends);

  } else if (strcmp(type, keyword_manual) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL);

  } else if (strcmp(type, keyword_field) == 0) {
    handle_userdata_field(node);

  } else {
    error(ERROR_SINGLETON, "AP_Objects only support renames, methods, field, semaphore or manual keywords (got %s)", type);
  }

  // check that we didn't just add 2 singleton flags
  int semaphore = (node->flags & UD_FLAG_SEMAPHORE)?1:0;
  semaphore += (node->flags & UD_FLAG_SCHEDULER_SEMAPHORE)?1:0;
  semaphore += (node->flags & UD_FLAG_SEMAPHORE_POINTER)?1:0;
  if (semaphore > 1) {
    error(ERROR_SINGLETON, "Taking multiple types of semaphore is prohibited");
  }

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Singleton contained an unexpected extra token: %s", state.token);
  }
}

void handle_global(void) {

  if (parsed_globals == NULL) {
    parsed_globals = (struct userdata *)allocate(sizeof(struct userdata));
    parsed_globals->ud_type = UD_GLOBAL;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_GLOBALS, "Expected a access type for global");
  }

  if (strcmp(type, keyword_manual) == 0) {
    handle_manual(parsed_globals, ALIAS_TYPE_MANUAL);
  } else {
    error(ERROR_GLOBALS, "globals only support manual keyword (got %s)", type);
  }

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_GLOBALS, "global contained an unexpected extra token: %s", state.token);
  }
}

void sanity_check_userdata(void) {
  struct userdata * node = parsed_userdata;
  while(node) {
    if ((node->fields == NULL) && (node->methods == NULL) && (node->method_aliases == NULL)) {
      error(ERROR_USERDATA, "Userdata %s has no fields or methods", node->name);
    }
    node = node->next;
  }
}

// set while the M4's local file is written: the M4 compiles every local type and learns the
// board's availability at run time (m4:bindings), so no #if guards are written
static int suppress_dependency_guards = 0;

void start_dependency(FILE *f, const char *dependency) {
  if ((dependency != NULL) && !suppress_dependency_guards) {
    fprintf(f, "#if %s\n", dependency);
  }
}

void end_dependency(FILE *f, const char *dependency) {
  if ((dependency != NULL) && !suppress_dependency_guards) {
    fprintf(f, "#endif // %s\n", dependency);
  }
}

int should_emit_creation(struct userdata * data) {
    if (data->creation || data->methods) {
      // Custom creation or methods, if not specifically disabled
      return !(data->creation && data->creation_args == -1);
    } else {
      // Don't expose creation function for items with only read-only fields
      struct userdata_field * field = data->fields;
      while(field) {
        if (field->access_flags & ACCESS_FLAG_WRITE) {
          return TRUE;
          break;
        }
        field = field->next;
      }
      return FALSE;
    }
}

void emit_headers(FILE *f) {
  struct header *node = headers;
  while (node) {
    start_dependency(f, node->dependency);
    fprintf(f, "#include <%s>\n", node->name);
    end_dependency(f, node->dependency);
    node = node->next;
  }
}

void emit_userdata_allocator(struct userdata *node) {
  start_dependency(source, node->dependency);
  // New method used internally
  fprintf(source, "%s * new_%s(lua_State *L) {\n", node->name, node->sanitized_name);
  fprintf(source, "    void *ud = lua_newuserdata(L, sizeof(%s));\n", node->name);
  // convince compiler to remove null check on the new, lua_newuserdata can't return null
  fprintf(source, "    if (!ud) { __builtin_unreachable(); } // elide constructor null check\n");
  fprintf(source, "    new (ud) %s();\n", node->name);
  fprintf(source, "    set_userdata_metatable(L, \"%s\");\n", node->rename ? node->rename :  node->name);
  fprintf(source, "    return (%s *)ud;\n", node->name);
  fprintf(source, "}\n");

  // New method used externally, includes argcheck, overridden by custom creation function if provided
  if (node->creation == NULL && should_emit_creation(node)) {
    fprintf(source, "\n");
    fprintf(source, "int lua_new_%s(lua_State *L) {\n", node->sanitized_name);

    // emit one time warning if augments are parsed
    fprintf(source, "    static bool warned = false;\n");
    fprintf(source, "    if (!warned && userdata_zero_arg_check(L)) {\n");
    fprintf(source, "        warned = true;\n");
    fprintf(source, "    }\n");

    fprintf(source, "    new_%s(L);\n", node->sanitized_name);
    fprintf(source, "    return 1;\n");
    fprintf(source, "}\n");
  }

  end_dependency(source, node->dependency);
  fprintf(source, "\n");
}

void emit_userdata_allocators(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    emit_userdata_allocator(node);
    node = node->next;
  }
}

void emit_ap_object_allocators(void) {
  struct userdata * node = parsed_ap_objects;
  while (node) {
    start_dependency(source, node->dependency);
    fprintf(source, "%s ** new_%s(lua_State *L) {\n", node->name, node->sanitized_name);
    fprintf(source, "    return (%s **)new_ap_object(L, sizeof(%s *), \"%s\");\n", node->name, node->name, node->name);
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_userdata_checker(struct userdata *node) {
  start_dependency(source, node->dependency);
  fprintf(source, "%s * check_%s(lua_State *L, int arg) {\n", node->name, node->sanitized_name);
  fprintf(source, "    return (%s *)luaL_checkudata(L, arg, \"%s\");\n", node->name, node->rename ? node->rename :  node->name);
  fprintf(source, "}\n");
  end_dependency(source, node->dependency);
  fprintf(source, "\n");
}

void emit_userdata_checkers(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    emit_userdata_checker(node);
    node = node->next;
  }
}

void emit_ap_object_checkers(void) {
  struct userdata * node = parsed_ap_objects;
  while (node) {
    start_dependency(source, node->dependency);
    fprintf(source, "%s ** check_%s(lua_State *L, int arg) {\n", node->name, node->sanitized_name);
    fprintf(source, "    return (%s **)check_ap_object(L, arg, \"%s\");\n", node->name, node->name);
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_singleton_checker(struct userdata *node) {
  if (!(node->flags & UD_FLAG_LITERAL) && (node->methods != NULL)) {
    start_dependency(source, node->dependency);
    fprintf(source, "%s * check_%s(lua_State *L) {\n", node->name, node->sanitized_name);
    if (node->get_string == NULL) {
      fprintf(source, "    %s * ud = %s::get_singleton();\n", node->name, node->name);
    } else {
      fprintf(source, "    %s * ud = %s;\n", node->name, node->get_string);
    }
    fprintf(source, "    if (ud == nullptr) {\n");
    fprintf(source, "        // This error will never return, so there is no danger of returning a nullptr\n");
    fprintf(source, "        not_supported_error(L, 1, \"%s\");\n", node->rename ? node->rename : node->name);
    fprintf(source, "    }\n");
    fprintf(source, "    return ud;\n");
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
  }
}

void emit_singleton_checkers(void) {
  struct userdata * node = parsed_singletons;
  while (node) {
    emit_singleton_checker(node);
    node = node->next;
  }
}

void emit_userdata_declarations(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    start_dependency(header, node->dependency);
    fprintf(header, "%s * new_%s(lua_State *L);\n", node->name, node->sanitized_name);
    if (node->creation == NULL && should_emit_creation(node)) {
      fprintf(header, "int lua_new_%s(lua_State *L);\n", node->sanitized_name);
    }
    fprintf(header, "%s * check_%s(lua_State *L, int arg);\n", node->name, node->sanitized_name);
    end_dependency(header, node->dependency);
    node = node->next;
  }
}

void emit_ap_object_declarations(void) {
  struct userdata * node = parsed_ap_objects;
  while (node) {
    start_dependency(header, node->dependency);
    fprintf(header, "%s ** new_%s(lua_State *L);\n", node->name, node->sanitized_name);
    fprintf(header, "%s ** check_%s(lua_State *L, int arg);\n", node->name, node->sanitized_name);
    end_dependency(header, node->dependency);
    node = node->next;
  }
}

#define NULLABLE_ARG_COUNT_BASE 5000
void emit_checker(const struct type t, int arg_number, int skipped, const char *indentation) {
  assert(indentation != NULL);

  if (arg_number > NULLABLE_ARG_COUNT_BASE) {
    error(ERROR_INTERNAL, "Can't handle more then %d arguments to a function", NULLABLE_ARG_COUNT_BASE);
  }

  if (t.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) {
    arg_number = arg_number + NULLABLE_ARG_COUNT_BASE;
    switch (t.type) {
      case TYPE_BOOLEAN:
        fprintf(source, "%sbool data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_FLOAT:
        fprintf(source, "%sfloat data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_INT8_T:
        fprintf(source, "%sint8_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_INT16_T:
        fprintf(source, "%sint16_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_INT32_T:
        fprintf(source, "%sint32_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_UINT8_T:
        fprintf(source, "%suint8_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_UINT16_T:
        fprintf(source, "%suint16_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "%suint32_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_AP_OBJECT:
      case TYPE_NONE:
      case TYPE_LITERAL:
        return; // nothing to do here, this should potentially be checked outside of this, but it makes an easier implementation to accept it
      case TYPE_STRING:
        fprintf(source, "%schar * data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_ENUM:
        fprintf(source, "%suint32_t data_%d {};\n", indentation, arg_number);
        break;
      case TYPE_USERDATA:
        fprintf(source, "%s%s data_%d {};\n", indentation, t.data.ud.name, arg_number);
        break;
    }
  } else {
    // handle this in four stages
    //   - figure out any relevant minimum values for range checking
    //   - emit a non down casted version
    //   - then run range checks
    //   - then cast down as appropriate

    // select minimums
    char * forced_min;
    char * forced_max;
    switch (t.type) {
      case TYPE_FLOAT:
        forced_min = "-INFINITY";
        forced_max = "INFINITY";
        break;
      case TYPE_INT8_T:
        forced_min = "INT8_MIN";
        forced_max = "INT8_MAX";
        break;
      case TYPE_INT16_T:
        forced_min = "INT16_MIN";
        forced_max = "INT16_MAX";
        break;
      case TYPE_INT32_T:
        forced_min = "INT32_MIN";
        forced_max = "INT32_MAX";
        break;
      case TYPE_UINT8_T:
        forced_min = "0";
        forced_max = "UINT8_MAX";
        break;
      case TYPE_UINT16_T:
        forced_min = "0";
        forced_max = "UINT16_MAX";
        break;
      case TYPE_UINT32_T:
        forced_min = "0U";
        forced_max = "UINT32_MAX";
        break;
      case TYPE_ENUM:
        forced_min = forced_max = NULL;
        break;
      case TYPE_NONE:
        return; // nothing to do here, this should potentially be checked outside of this, but it makes an easier implementation to accept it
      case TYPE_AP_OBJECT:
      case TYPE_STRING:
      case TYPE_BOOLEAN:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        // these don't get range checked, so skip the raw_data phase
        assert(t.range == NULL); // we should have caught this during the parse phase
        break;
    }

    // non down cast
    char * type_name;
    char * get_name;
    char * get_and_check_name;
    switch (t.type) {
      case TYPE_FLOAT:
        type_name = "float";
        get_name = "luaL_checknumber";
        get_and_check_name = "get_number";
        break;
      case TYPE_INT8_T:
        type_name = (t.range == NULL)?"int8_t":"lua_Integer";
        get_name = "get_int8_t";
        get_and_check_name = "get_integer";
        break;
      case TYPE_INT16_T:
        type_name = (t.range == NULL)?"int16_t":"lua_Integer";
        get_name = "get_int16_t";
        get_and_check_name = "get_integer";
        break;
      case TYPE_UINT8_T:
        type_name = (t.range == NULL)?"uint8_t":"lua_Integer";
        get_name = "get_uint8_t";
        get_and_check_name = "get_integer";
        break;
      case TYPE_UINT16_T:
        type_name = (t.range == NULL)?"uint16_t":"lua_Integer";
        get_name = "get_uint16_t";
        get_and_check_name = "get_integer";
        break;
      case TYPE_INT32_T:
      case TYPE_ENUM:
        type_name = "lua_Integer";
        get_name = "luaL_checkinteger";
        get_and_check_name = "get_integer";
        break;
      case TYPE_UINT32_T:
        type_name = "uint32_t";
        get_name = "coerce_to_uint32_t";
        get_and_check_name = "get_uint32";
        break;
      case TYPE_AP_OBJECT:
      case TYPE_NONE:
      case TYPE_STRING:
      case TYPE_BOOLEAN:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        // these don't get range checked, so skip the raw_data phase
        assert(t.range == NULL); // we should have caught this during the parse phase
        break;
    }


    switch (t.type) {
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_ENUM:
      case TYPE_UINT32_T:
        if (t.range == NULL) {
            fprintf(source, "%sconst %s raw_data_%d = %s(L, %d);\n", indentation, type_name, arg_number, get_name, arg_number - skipped);
        } else {
          if ((forced_min != NULL) && (forced_max != NULL)) {
            fprintf(source, "%sconst %s raw_data_%d = %s(L, %d, MAX(%s, %s), MIN(%s, %s));\n", indentation, type_name, arg_number, get_and_check_name, arg_number - skipped, t.range->low, forced_min,  t.range->high, forced_max);
          } else {
            char * cast_target = "";

            switch (t.type) {
              case TYPE_FLOAT:
                cast_target = "float";
                break;
              case TYPE_INT8_T:
              case TYPE_INT16_T:
              case TYPE_INT32_T:
              case TYPE_UINT8_T:
              case TYPE_UINT16_T:
              case TYPE_ENUM:
                cast_target = "int32_t";
                break;
              case TYPE_UINT32_T:
                cast_target = "uint32_t";
                break;
              case TYPE_AP_OBJECT:
              case TYPE_NONE:
              case TYPE_STRING:
              case TYPE_BOOLEAN:
              case TYPE_USERDATA:
              case TYPE_LITERAL:
                assert(t.range == NULL); // we should have caught this during the parse phase
                break;
            }
            fprintf(source, "%sconst %s raw_data_%d = %s(L, %d, static_cast<%s>(%s), static_cast<%s>(%s));\n", indentation, type_name, arg_number, get_and_check_name, arg_number - skipped, cast_target, t.range->low, cast_target, t.range->high);

          }
        }

        break;
      case TYPE_AP_OBJECT:
      case TYPE_NONE:
      case TYPE_STRING:
      case TYPE_BOOLEAN:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        break;
    }

    // down cast
    switch (t.type) {
      case TYPE_FLOAT:
        // this is a trivial transformation, trust the compiler to resolve it for us
        fprintf(source, "%sconst float data_%d = raw_data_%d;\n", indentation, arg_number, arg_number);
        break;
      case TYPE_INT8_T:
        fprintf(source, "%sconst int8_t data_%d = static_cast<int8_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_INT16_T:
        fprintf(source, "%sconst int16_t data_%d = static_cast<int16_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_INT32_T:
        fprintf(source, "%sconst int32_t data_%d = raw_data_%d;\n", indentation, arg_number, arg_number);
        break;
      case TYPE_UINT8_T:
        fprintf(source, "%sconst uint8_t data_%d = static_cast<uint8_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_UINT16_T:
        fprintf(source, "%sconst uint16_t data_%d = static_cast<uint16_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "%sconst uint32_t data_%d = static_cast<uint32_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_BOOLEAN:
        fprintf(source, "%sconst bool data_%d = static_cast<bool>(lua_toboolean(L, %d));\n", indentation, arg_number, arg_number - skipped);
        break;
      case TYPE_STRING:
        fprintf(source, "%sconst char * data_%d = luaL_checkstring(L, %d);\n", indentation, arg_number, arg_number - skipped);
        break;
      case TYPE_ENUM:
        fprintf(source, "%sconst %s data_%d = static_cast<%s>(raw_data_%d);\n", indentation, t.data.enum_name, arg_number, t.data.enum_name, arg_number);
        break;
      case TYPE_USERDATA:
        fprintf(source, "%s%s & data_%d = *check_%s(L, %d);\n", indentation, t.data.ud.name, arg_number, t.data.ud.sanitized_name, arg_number - skipped);
        break;
      case TYPE_AP_OBJECT:
        fprintf(source, "%s%s * data_%d = *check_%s(L, %d);\n", indentation, t.data.ud.name, arg_number, t.data.ud.sanitized_name, arg_number - skipped);
        break;
      case TYPE_LITERAL:
        // literals are expected to be done directly later
        break;
      case TYPE_NONE:
        // nothing to do, we've either already emitted a reasonable value, or returned
        break;
    }
  }
}

void emit_field(const struct userdata_field *field, const char* object_name, const char* object_access) {
  // if there is only one access type there is no need to deal with multiple arguments in a switch
  int use_switch = TRUE;
  int args = 1;
  char *indent = "            ";
  if (((field->access_flags & ACCESS_FLAG_READ) == 0) || ((field->access_flags & ACCESS_FLAG_WRITE) == 0)) {
    use_switch = FALSE;
    indent = "    ";
    if (field->access_flags & ACCESS_FLAG_WRITE) {
      args = 2;
    }
  }

  char *index_string = "";
  int write_arg_number = 2;
  if (field->array_len != NULL) {
    index_string = "[index]";
    write_arg_number = 3;

    if (!use_switch) {
      fprintf(source, "    binding_argcheck(L, %d);\n",args+1);
    }

    fprintf(source, "\n    const lua_Integer raw_index = get_integer(L, 2, 0, MIN(%s-1, UINT8_MAX));\n",field->array_len);
    fprintf(source, "    const uint8_t index = static_cast<uint8_t>(raw_index);\n\n");

    if (use_switch) {
      fprintf(source, "    switch(lua_gettop(L)-1) {\n");
    }

  } else if (use_switch) {
    fprintf(source, "    switch(lua_gettop(L)) {\n");
  } else {
    fprintf(source, "    binding_argcheck(L, %d);\n",args);
  }

  int valid_mask = (field->type.valid_mask.name != NULL) && (field->type.valid_mask.value != NULL);

  if (field->access_flags & ACCESS_FLAG_READ) {
    if (use_switch) {
      fprintf(source, "        case 1:\n");
    }

    // Check if field is valid
    if (valid_mask) {
        fprintf(source, "%sif ((%s%s%s & %s) == 0) {\n", indent, object_name, object_access, field->type.valid_mask.name, field->type.valid_mask.value);
        fprintf(source, "%s    return 0;\n", indent);
        fprintf(source, "%s}\n", indent);
    }

    switch (field->type.type) {
      case TYPE_BOOLEAN:
        fprintf(source, "%slua_pushinteger(L, %s%s%s%s);\n", indent, object_name, object_access, field->name, index_string);
        break;
      case TYPE_FLOAT:
        fprintf(source, "%slua_pushnumber(L, %s%s%s%s);\n", indent, object_name, object_access, field->name, index_string);
        break;
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
        fprintf(source, "%slua_pushinteger(L, %s%s%s%s);\n", indent, object_name, object_access, field->name, index_string);
        break;
      case TYPE_ENUM:
        fprintf(source, "%slua_pushinteger(L, static_cast<int32_t>(%s%s%s%s));\n", indent, object_name, object_access, field->name, index_string);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "%s*new_uint32_t(L) = %s%s%s%s;\n", indent, object_name, object_access, field->name, index_string);
        break;
      case TYPE_NONE:
        error(ERROR_INTERNAL, "Can't access a NONE field");
        break;
      case TYPE_LITERAL:
        error(ERROR_INTERNAL, "Can't access a literal field");
        break;
      case TYPE_STRING:
        fprintf(source, "%slua_pushstring(L, %s%s%s%s);\n", indent, object_name, object_access, field->name, index_string);
        break;
      case TYPE_USERDATA:
          fprintf(source, "%s*new_%s(L) = %s%s%s%s;\n", indent, field->type.data.ud.sanitized_name, object_name, object_access, field->name, index_string);
        break;
      case TYPE_AP_OBJECT: // FIXME: collapse the identical cases here, and use the type string function
        error(ERROR_USERDATA, "AP_Object does not currently support access to userdata field's");
        break;
    }
    fprintf(source, "%sreturn 1;\n", indent);
  }

  if (field->access_flags & ACCESS_FLAG_WRITE) {
    if (use_switch) {
      fprintf(source, "        case 2: {\n");
    }
    emit_checker(field->type, write_arg_number, 0, indent);

    // set field valid
    if (valid_mask) {
      fprintf(source, "%s%s%s%s |= %s;\n", indent, object_name, object_access, field->type.valid_mask.name, field->type.valid_mask.value);
    }

    fprintf(source, "%s%s%s%s%s = data_%i;\n", indent, object_name, object_access, field->name, index_string, write_arg_number);
    fprintf(source, "%sreturn 0;\n", indent);
    if (use_switch) {
      fprintf(source, "         }\n");
    }
  }

  if (use_switch) {
    fprintf(source, "        default:\n");
    fprintf(source, "            return field_argerror(L); // too many arguments\n");
    fprintf(source, "    }\n");
  }

  fprintf(source, "}\n\n");
}

void emit_userdata_field(const struct userdata *data, const struct userdata_field *field) {
  start_dependency(source, field->dependency);
  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanitized_name, field->name);
  fprintf(source, "    %s *ud = check_%s(L, 1);\n", data->name, data->sanitized_name);
  emit_field(field, "ud", "->");
  end_dependency(source, field->dependency);
}

void emit_userdata_fields() {
  struct userdata * node = parsed_userdata;
  while(node) {
    struct userdata_field *field = node->fields;
    if (field) {
      start_dependency(source, node->dependency);
      while(field) {
        emit_userdata_field(node, field);
        field = field->next;
      }
      end_dependency(source, node->dependency);
    }
    node = node->next;
  }
}

void emit_singleton_field(const struct userdata *data, const struct userdata_field *field) {
  start_dependency(source, field->dependency);

  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanitized_name, field->name);

  // emit comments on expected arg/type
  if (!(data->flags & UD_FLAG_LITERAL)) {
      // fetch and check the singleton pointer
      fprintf(source, "    %s * ud = check_%s(L);\n", data->name, data->sanitized_name);
  }
  const char *ud_name = (data->flags & UD_FLAG_LITERAL)?data->name:"ud";
  const char *ud_access = (data->flags & UD_FLAG_REFERENCE)?".":"->";

  emit_field(field, ud_name, ud_access);

  end_dependency(source, field->dependency);
}

void emit_singleton_fields() {
  struct userdata * node = parsed_singletons;
  while(node) {
    struct userdata_field *field = node->fields;
    if (field) {
      start_dependency(source, node->dependency);
      while(field) {
        emit_singleton_field(node, field);
        field = field->next;
      }
      end_dependency(source, node->dependency);
    }
    node = node->next;
  }
}

void emit_ap_object_field(const struct userdata *data, const struct userdata_field *field) {
  start_dependency(source, field->dependency);
  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanitized_name, field->name);
  fprintf(source, "    %s *ud = *check_%s(L, 1);\n", data->name, data->sanitized_name);
  emit_field(field, "ud", "->");
  end_dependency(source, field->dependency);
}

void emit_ap_object_fields() {
  struct userdata * node = parsed_ap_objects;
  while(node) {
    struct userdata_field *field = node->fields;
    if (field) {
      start_dependency(source, node->dependency);
      while(field) {
        emit_ap_object_field(node, field);
        field = field->next;
      }
      end_dependency(source, node->dependency);
    }
    node = node->next;
  }
}

// emit references functions for a call, return the number of arguments added
int emit_references(const struct argument *arg, const char * tab) {
  int arg_index = NULLABLE_ARG_COUNT_BASE + 2;
  int return_count = 0;
  // count arguments to return so we know if we need to check the stack
  const struct argument *count_arg = arg;
  while (count_arg != NULL) {
    if (count_arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) {
      return_count++;
    }
    count_arg = count_arg->next;
  }
  // add one to have a spare stack slot for userdata creation funcs
  fprintf(source, "#if %d > LUA_MINSTACK\n", return_count+1);
  fprintf(source, "%sluaL_checkstack(L, %d, nullptr);\n", tab, return_count+1);
  fprintf(source, "#endif\n\n");
  while (arg != NULL) {
    if (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) {
      switch (arg->type.type) {
        case TYPE_BOOLEAN:
          fprintf(source, "%slua_pushboolean(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_FLOAT:
          fprintf(source, "%slua_pushnumber(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_INT8_T:
        case TYPE_INT16_T:
        case TYPE_INT32_T:
        case TYPE_UINT8_T:
        case TYPE_UINT16_T:
        case TYPE_ENUM:
          fprintf(source, "%slua_pushinteger(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_UINT32_T:
          fprintf(source, "%s*new_uint32_t(L) = data_%d;\n", tab, arg_index);
          break;
        case TYPE_STRING:
          fprintf(source, "%slua_pushstring(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_USERDATA:
          fprintf(source, "%s*new_%s(L) = data_%d;\n", tab, arg->type.data.ud.sanitized_name, arg_index);
          break;
        case TYPE_NONE:
          error(ERROR_INTERNAL, "Attempted to emit a nullable or reference argument of type none");
          break;
        case TYPE_LITERAL:
          error(ERROR_INTERNAL, "Attempted to make a nullable or reference literal");
          break;
        case TYPE_AP_OBJECT: // FIXME: collapse these to a single failure case
          error(ERROR_INTERNAL, "Attempted to make a nullable or reference ap_object");
          break;
      }
    }
    arg_index++;
    arg = arg->next;
  }
  return return_count;
}

void emit_userdata_method(const struct userdata *data, struct method *method) {
  int arg_count = 1;

  if (!stats_ids_frozen) {
    method->stats_id = assign_stats_id(data->rename ? data->rename : data->name, method->rename ? method->rename : method->name);
  }

  start_dependency(source, data->dependency);
  start_dependency(source, method->dependency);


  // bind ud early if it's a singleton, so that we can use it in the range checks
  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanitized_name, method->sanitized_name);
  // emit comments on expected arg/type
  struct argument *arg = method->arguments;

  // emit warning if configured
  if (method->deprecate != NULL) {
    fprintf(source, "    static bool warned = false;\n");
    fprintf(source, "    if (!warned) {\n");
    fprintf(source, "        lua_scripts::set_and_print_new_error_message(MAV_SEVERITY_WARNING, \"%s:%s %s\");\n", data->rename, method->rename ? method->rename : method->name, method->deprecate);
    fprintf(source, "        warned = true;\n");
    fprintf(source, "    }\n\n");
  }

  // sanity check number of args called with
  arg_count = 1;
  while (arg != NULL) {
    if (!(arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) && !(arg->type.type == TYPE_LITERAL)) {
      arg_count++;
    }
    arg = arg->next;
  }
  fprintf(source, "    binding_argcheck(L, %d);\n", arg_count);

  switch (data->ud_type) {
    case UD_USERDATA:
      // extract the userdata
      fprintf(source, "    %s * ud = check_%s(L, 1);\n", data->name, data->sanitized_name);
      break;
    case UD_SINGLETON:
      if (!(data->flags & UD_FLAG_LITERAL)) {
        // fetch and check the singleton pointer
        fprintf(source, "    %s * ud = check_%s(L);\n", data->name, data->sanitized_name);
      }
      break;
    case UD_GLOBAL:
      break;
    case UD_AP_OBJECT:
      // extract the userdata, it was a pointer, so we need to grab it
      fprintf(source, "    %s * ud = *check_%s(L, 1);\n", data->name, data->sanitized_name);
      break;
  }

  // extract the arguments
  arg = method->arguments;
  arg_count = 2;
  int skipped = 0;
  while (arg != NULL) {
    if (arg->type.type != TYPE_LITERAL) {
      // emit_checker will emit a nullable argument for us
      emit_checker(arg->type, arg_count, skipped, "    ");
      arg_count++;
    }
    if (//arg->type.type == TYPE_LITERAL ||
        arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) {
      skipped++;
    }
    arg = arg->next;
  }

  const char *ud_name = (data->flags & UD_FLAG_LITERAL)?data->name:"ud";
  const char *ud_access = (data->flags & UD_FLAG_REFERENCE)?".":"->";

  const int has_semaphore = data->flags & (UD_FLAG_SEMAPHORE | UD_FLAG_SEMAPHORE_POINTER | UD_FLAG_SCHEDULER_SEMAPHORE);
  if (has_semaphore) {
    fprintf(source, "    BINDING_SEM_WAIT_BEGIN();\n");
  }
  if (data->flags & UD_FLAG_SEMAPHORE) {
    fprintf(source, "    %s%sget_semaphore().take_blocking();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SEMAPHORE_POINTER) {
    fprintf(source, "    %s%sget_semaphore()->take_blocking();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SCHEDULER_SEMAPHORE) {
    fprintf(source, "#if AP_SCHEDULER_ENABLED\n");
    fprintf(source, "    AP::scheduler().get_semaphore().take_blocking();\n");
    fprintf(source, "#endif\n");
  }
  if (has_semaphore) {
    fprintf(source, "    BINDING_SEM_WAIT_END(%d);\n", method->stats_id);
  }

  int static_cast = TRUE;

  switch (method->return_type.type) {
    case TYPE_STRING:
      fprintf(source, "    const char * data = %s%s%s(", ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_ENUM:
      fprintf(source, "    const %s &data = %s%s%s(", method->return_type.data.enum_name, ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_USERDATA:
      if (strcmp(method->name, "copy") == 0) {
          // special case for copy method
          fprintf(source, "    const %s data = (*%s", method->return_type.data.ud.name, ud_name);
      } else {
          fprintf(source, "    const %s &data = %s%s%s(", method->return_type.data.ud.name, ud_name, ud_access, method->name);
      }
      static_cast = FALSE;
      break;
    case TYPE_AP_OBJECT:
      fprintf(source, "    %s *data = %s%s%s(", method->return_type.data.ud.name, ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_NONE:
      fprintf(source, "    %s%s%s(", ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_LITERAL:
      error(ERROR_USERDATA, "Can't return a literal from a method");
      break;
    case TYPE_BOOLEAN:
    case TYPE_FLOAT:
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_UINT32_T:
      break;
  }

  if (static_cast) {
    char *var_type_name;
    switch (method->return_type.type) {
      case TYPE_BOOLEAN:
        var_type_name = "bool";
        break;
      case TYPE_FLOAT:
        var_type_name = "float";
        break;
      case TYPE_INT8_T:
        var_type_name = "int8_t";
        break;
      case TYPE_INT16_T:
        var_type_name = "int16_t";
        break;
      case TYPE_INT32_T:
        var_type_name = "int32_t";
        break;
      case TYPE_UINT8_T:
        var_type_name = "uint8_t";
        break;
      case TYPE_UINT16_T:
        var_type_name = "uint16_t";
        break;
      case TYPE_UINT32_T:
        var_type_name = "uint32_t";
        break;
    case TYPE_STRING:
    case TYPE_ENUM:
    case TYPE_USERDATA:
    case TYPE_AP_OBJECT:
    case TYPE_NONE:
    case TYPE_LITERAL:
        error(ERROR_USERDATA, "Unexpected type");
        break;
    }
    fprintf(source, "    const %s data = static_cast<%s>(%s%s%s(", var_type_name, var_type_name, ud_name, ud_access, method->name);
  }

  if (arg_count != 2) {
    fprintf(source, "\n");
  }

  arg = method->arguments;
  arg_count = 2;
  while (arg != NULL) {
    switch (arg->type.type) {
      case TYPE_BOOLEAN:
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_STRING:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
      case TYPE_ENUM:
      case TYPE_USERDATA:
      case TYPE_AP_OBJECT:
        fprintf(source, "            %sdata_%d", (arg->type.access == ACCESS_REFERENCE)?"&":"", arg_count + ((arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) ? NULLABLE_ARG_COUNT_BASE : 0));
        break;
      case TYPE_LITERAL:
        fprintf(source, "            %s", arg->type.data.literal);
        break;
      case TYPE_NONE:
        error(ERROR_INTERNAL, "Can't pass nil as an argument");
        break;
    }
    if (arg->type.type != TYPE_LITERAL) {
      arg_count++;
    }
    arg = arg->next;
    if (arg != NULL) {
            fprintf(source, ",\n");
    }
  }
  if (static_cast) {
    fprintf(source, "%s));\n\n", "");
  } else {
    fprintf(source, "%s);\n\n", "");
  }

  if (data->flags & UD_FLAG_SEMAPHORE) {
    fprintf(source, "    %s%sget_semaphore().give();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SEMAPHORE_POINTER) {
    fprintf(source, "    %s%sget_semaphore()->give();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SCHEDULER_SEMAPHORE) {
    fprintf(source, "#if AP_SCHEDULER_ENABLED\n");
    fprintf(source, "    AP::scheduler().get_semaphore().give();\n");
    fprintf(source, "#endif\n");
  }

  // we need to emit out reference arguments, iterate the args again, creating and copying objects, while keeping a new count
  int return_count = 1; 
  if (method->flags & TYPE_FLAGS_REFERENCE) {
    arg = method->arguments;
    // number of arguments to return
    return_count += emit_references(arg,"    ");
  }

  switch (method->return_type.type) {
    case TYPE_BOOLEAN:
      if (method->flags & TYPE_FLAGS_NULLABLE) {
        fprintf(source, "    if (data) {\n");
        // we need to emit out nullable arguments, iterate the args again, creating and copying objects, while keeping a new count
        arg = method->arguments;
        return_count = emit_references(arg,"        ");
        fprintf(source, "        return %d;\n", return_count);
        fprintf(source, "    }\n");
        fprintf(source, "    return 0;\n");
      } else {
        fprintf(source, "    lua_pushboolean(L, data);\n");
      }
      break;
    case TYPE_FLOAT:
      fprintf(source, "    lua_pushnumber(L, data);\n");
      break;
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_ENUM:
      fprintf(source, "    lua_pushinteger(L, data);\n");
      break;
    case TYPE_UINT32_T:
      fprintf(source, "        *new_uint32_t(L) = data;\n");
      break;
    case TYPE_STRING:
      fprintf(source, "    lua_pushstring(L, data);\n");
      break;
    case TYPE_USERDATA:
      fprintf(source, "    *new_%s(L) = data;\n", method->return_type.data.ud.sanitized_name);
      break;
    case TYPE_AP_OBJECT:
      fprintf(source, "    if (data == NULL) {\n");
      fprintf(source, "        return 0;\n");
      fprintf(source, "    }\n");
      fprintf(source, "    *new_%s(L) = data;\n", method->return_type.data.ud.sanitized_name);
      break;
    case TYPE_NONE:
    case TYPE_LITERAL:
      // no return value, so don't worry about pushing a value
      return_count--;
      break;
  }

  if ((method->return_type.type != TYPE_BOOLEAN) || ((method->flags & TYPE_FLAGS_NULLABLE) == 0)) {
      fprintf(source, "    return %d;\n", return_count);
  }

  fprintf(source, "}\n");
  end_dependency(source, method->dependency);
  end_dependency(source, data->dependency);
  fprintf(source, "\n");

}

const char * get_name_for_operation(enum operator_type op) {
  switch (op) {
    case OP_ADD:
      return "__add";
    case OP_SUB:
      return "__sub";
    case OP_MUL:
      return "__mul";
    case OP_DIV:
      return "__div";
    case OP_MOD:
      return "__mod";
    case OP_BAND:
      return "__band";
    case OP_BOR:
      return "__bor";
    case OP_BXOR:
      return "__bxor";
    case OP_SHL:
      return "__shl";
    case OP_SHR:
      return "__shr";
    case OP_EQ:
      return "__eq";
    case OP_LT:
      return "__lt";
    case OP_LE:
      return "__le";
    case OP_BNOT:
      return "__bnot";
    case OP_MANUAL:
    case OP_LAST:
      return NULL;
  }
  return NULL;
}

const char * get_sym_for_operation(enum operator_type op) {
  switch (op) {
    case OP_ADD:
      return "+";
    case OP_SUB:
      return "-";
    case OP_MUL:
      return "*";
    case OP_DIV:
      return "/";
    case OP_MOD:
      return "%";
    case OP_BAND:
      return "&";
    case OP_BOR:
      return "|";
    case OP_BXOR:
      return "^";
    case OP_SHL:
      return "<<";
    case OP_SHR:
      return ">>";
    case OP_EQ:
      return "==";
    case OP_LT:
      return "<";
    case OP_LE:
      return "<=";
    case OP_BNOT:
      return "~";
    case OP_MANUAL:
    case OP_LAST:
      return NULL;
  }
  return NULL;
}

int operation_is_bool(enum operator_type op) {
  switch (op) {
    case OP_ADD:
    case OP_SUB:
    case OP_MUL:
    case OP_DIV:
    case OP_MOD:
    case OP_BAND:
    case OP_BOR:
    case OP_BXOR:
    case OP_SHL:
    case OP_SHR:
    case OP_BNOT:
    case OP_MANUAL:
    case OP_LAST:
      return FALSE;

    case OP_EQ:
    case OP_LT:
    case OP_LE:
      return TRUE;
  }
  return FALSE;
}

int operation_is_unary(enum operator_type op) {
  switch (op) {
    case OP_ADD:
    case OP_SUB:
    case OP_MUL:
    case OP_DIV:
    case OP_MOD:
    case OP_BAND:
    case OP_BOR:
    case OP_BXOR:
    case OP_SHL:
    case OP_SHR:
    case OP_EQ:
    case OP_LT:
    case OP_LE:
    case OP_MANUAL:
    case OP_LAST:
      return FALSE;

    case OP_BNOT:
      return TRUE;
  }
  return FALSE;
}

void emit_operators(struct userdata *data) {
  trace(TRACE_USERDATA, "Emitting operators for %s", data->name);

  assert(data->ud_type == UD_USERDATA);

  start_dependency(source, data->dependency);

  for (uint32_t i = 1; i < OP_LAST; i = (i << 1)) {
    const enum operator_type type = (data->operations) & i;

    const char * op_name = get_name_for_operation(type);
    if (op_name == NULL) {
      continue;
    }

    const char * op_sym = get_sym_for_operation(type);
    if (op_sym == NULL) {
      error(ERROR_USERDATA, "No symbol for %s operation %u", data->name, type);
    }

    // The generated check functions return pointers, the manual getters return a value directly
    const int have_getter = data->operator_getter != NULL;
    const char * access = have_getter ? "" : "*";
    const char * getter_prefix = have_getter ? "" : "check_";
    const char * getter = have_getter ? data->operator_getter : data->sanitized_name;

    fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanitized_name, op_name);
    // check number of arguments
    fprintf(source, "    binding_argcheck(L, 2);\n");
    // check the pointers
    fprintf(source, "    %s %sud = %s%s(L, 1);\n", data->name, access, getter_prefix, getter);

    if (!operation_is_unary(type)) {
      // Need two values
      fprintf(source, "    %s %sud2 = %s%s(L, 2);\n", data->name, access, getter_prefix, getter);

      if (operation_is_bool(type)) {
        // Return bool
        fprintf(source, "    lua_pushboolean(L, (%sud) %s (%sud2));\n", access, op_sym, access);

      } else {
        // Return same type
        // create a container for the result
        fprintf(source, "    *new_%s(L) = (%sud) %s (%sud2);\n", data->sanitized_name, access, op_sym, access);
      }

    } else {
      // Only a single value, lua pushes the same value onto the stack twice, so we still check for 2 arguments
      fprintf(source, "    *new_%s(L) = %s (%sud);\n", data->sanitized_name, op_sym, access);

    }

    // return the first pointer
    fprintf(source, "    return 1;\n");
    fprintf(source, "}\n\n");

  }

  end_dependency(source, data->dependency);
}

void emit_methods(struct userdata *node) {
  while(node) {
    // methods
    struct method *method = node->methods;
    while(method) {
      emit_userdata_method(node, method);
      method = method->next;
    }

    // operators
    if (node->operations) {
      emit_operators(node);
    }
    node = node->next;
  }
}

void emit_enum(struct userdata * data) {
    fprintf(source, "const struct userdata_enum %s_enums[] = {\n", data->sanitized_name);
    struct userdata_enum *ud_enum = data->enums;
    while (ud_enum != NULL) {
      fprintf(source, "    {\"%s\", %s::%s},\n", ud_enum->name, data->name, ud_enum->name);
      ud_enum = ud_enum->next;
    }
    fprintf(source, "};\n\n");
}

void emit_index_node(struct userdata *node) {
  start_dependency(source, node->dependency);

  fprintf(source, "const luaL_Reg %s_meta[] = {\n", node->sanitized_name);

  struct method *method = node->methods;
  while (method) {
    start_dependency(source, method->dependency);
    fprintf(source, "    {\"%s\", BINDING_FN(%d, %s_%s)},\n", method->rename ? method->rename :  method->name, method->stats_id, node->sanitized_name, method->name);
    end_dependency(source, method->dependency);
    method = method->next;
  }

  struct userdata_field *field = node->fields;
  while(field) {
    start_dependency(source, field->dependency);
    fprintf(source, "    {\"%s\", %s_%s},\n", field->rename ? field->rename : field->name, node->sanitized_name, field->name);
    end_dependency(source, field->dependency);
    field = field->next;
  }

  struct method_alias *alias = node->method_aliases;
  while(alias) {
    start_dependency(source, alias->dependency);
    if (alias->type == ALIAS_TYPE_MANUAL) {
      if (!stats_ids_frozen) {
        alias->stats_id = assign_stats_id(node->rename ? node->rename : node->name, alias->alias);
      }
      fprintf(source, "    {\"%s\", BINDING_FN(%d, %s)},\n", alias->alias, alias->stats_id, alias->name);
    } else if (alias->type == ALIAS_TYPE_NONE) {
      // share the target method's id so calls and semaphore waits land in one entry
      alias->stats_id = -1;
      for (struct method *target = node->methods; target != NULL; target = target->next) {
        if (strcmp(target->name, alias->name) == 0) {
          alias->stats_id = target->stats_id;
          break;
        }
      }
      if (alias->stats_id < 0) {
        error(ERROR_GENERAL, "Alias %s has no target method %s", alias->alias, alias->name);
      }
      fprintf(source, "    {\"%s\", BINDING_FN(%d, %s_%s)},\n", alias->alias, alias->stats_id, node->sanitized_name, alias->name);
    }
    end_dependency(source, alias->dependency);
    alias = alias->next;
  }

  fprintf(source, "};\n\n");

  if (node->operations) {
    fprintf(source, "const luaL_Reg %s_operators[] = {\n", node->sanitized_name);
    for (uint32_t i = 1; i < OP_LAST; i = (i << 1)) {
      const char * op_name = get_name_for_operation((node->operations) & i);
      if (op_name == NULL) {
        continue;
      }
      fprintf(source, "    {\"%s\", %s_%s},\n", op_name, node->sanitized_name, op_name);
    }
    struct method_alias *alias = node->method_aliases;
    while(alias) {
      if (alias->type == ALIAS_TYPE_MANUAL_OPERATOR) {
        fprintf(source, "    {\"%s\", %s},\n", alias->alias, alias->name);
      }
      alias = alias->next;
    }
    fprintf(source, "    {NULL, NULL},\n");
    fprintf(source, "};\n\n");
  }

  if (node->enums != NULL) {
    emit_enum(node);
  }

  fprintf(source, "static int %s_index(lua_State *L) {\n", node->sanitized_name);
  fprintf(source, "    return load_function(L,%s_meta,ARRAY_SIZE(%s_meta))",node->sanitized_name,node->sanitized_name);
  if (node->enums != NULL) {
    fprintf(source, " || load_enum(L,%s_enums,ARRAY_SIZE(%s_enums))",node->sanitized_name,node->sanitized_name);
  }
  fprintf(source, ";\n");
  fprintf(source, "}\n");
  end_dependency(source, node->dependency);
  fprintf(source, "\n");
}

void emit_index(struct userdata *head) {
  struct userdata * node = head;
  while(node) {
    emit_index_node(node);
    node = node->next;
  }
}

void emit_type_index(struct userdata * data, char * meta_name) {
  fprintf(source, "const struct luaL_Reg %s_fun[] = {\n", meta_name);
  while (data) {
    start_dependency(source, data->dependency);
    fprintf(source, "    {\"%s\", %s_index},\n", data->rename ? data->rename : data->name, data->sanitized_name);
    end_dependency(source, data->dependency);
    data = data->next;
  }
  fprintf(source, "};\n\n");
}

void emit_type_index_with_operators(struct userdata * data, char * meta_name) {
  fprintf(source, "const struct userdata_meta %s_fun[] = {\n", meta_name);
  while (data) {
    start_dependency(source, data->dependency);
    if (data->operations == 0) {
      fprintf(source, "    {\"%s\", %s_index, nullptr},\n", data->rename ? data->rename : data->name, data->sanitized_name);
    } else {
      fprintf(source, "    {\"%s\", %s_index, %s_operators},\n", data->rename ? data->rename : data->name, data->sanitized_name, data->sanitized_name);
    }
    end_dependency(source, data->dependency);
    data = data->next;
  }
  fprintf(source, "};\n\n");
}

void emit_loaders(void) {
  emit_type_index_with_operators(parsed_userdata, "userdata");
  emit_type_index(parsed_singletons, "singleton");
  emit_type_index(parsed_ap_objects, "ap_object");
}

void emit_userdata_new_funcs(void) {
  struct userdata *data = parsed_userdata;
  fprintf(source, "const struct userdata {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    const lua_CFunction fun;\n");
  fprintf(source, "} new_userdata[] = {\n");
  while (data) {
    if (should_emit_creation(data)) {
      start_dependency(source, data->dependency);
      if (data->creation) {
        // expose custom creation function to user (not used internally)
        fprintf(source, "    {\"%s\", %s},\n", data->rename ? data->rename :  data->name, data->creation);
      } else {
        fprintf(source, "    {\"%s\", lua_new_%s},\n", data->rename ? data->rename :  data->name, data->sanitized_name);
      }
      end_dependency(source, data->dependency);
    }
    data = data->next;
  }
  if (parsed_globals) {
    struct method_alias *manual_aliases = parsed_globals->method_aliases;
    while (manual_aliases) {
      if (manual_aliases->type != ALIAS_TYPE_MANUAL) {
        error(ERROR_GLOBALS, "Globals only support manual methods");
      }
      start_dependency(source, manual_aliases->dependency);
      manual_aliases->stats_id = assign_stats_id("global", manual_aliases->alias);
      fprintf(source, "    {\"%s\", BINDING_FN(%d, %s)},\n", manual_aliases->alias, manual_aliases->stats_id, manual_aliases->name);
      end_dependency(source, manual_aliases->dependency);
      manual_aliases = manual_aliases->next;
    }
  }
  fprintf(source, "};\n\n");
}

void emit_docs_type(struct type type, const char *prefix, const char *suffix) {
  switch (type.type) {
    case TYPE_BOOLEAN:
      fprintf(docs, "%s boolean%s", prefix, suffix);
      break;
    case TYPE_FLOAT:
      fprintf(docs, "%s number%s", prefix, suffix);
      break;
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_ENUM:
      fprintf(docs, "%s integer%s", prefix, suffix);
      break;
    case TYPE_STRING:
      fprintf(docs, "%s string%s", prefix, suffix);
      break;
    case TYPE_UINT32_T:
      fprintf(docs, "%s uint32_t_ud%s", prefix, suffix);
      break;
    case TYPE_USERDATA: {
      // userdata may have rename
      struct userdata *data = parsed_userdata;
      int found = 0;
      while (data) {
        if (strcmp(type.data.ud.sanitized_name, data->sanitized_name) == 0) {
          found = 1;
          break;
        }
        data = data->next;
      }
      if (found == 0) {
        error(ERROR_GENERAL, "Could not find userdata %s", type.data.ud.sanitized_name);
      }
      fprintf(docs, "%s %s_ud%s", prefix, data->rename ? data->rename : data->sanitized_name, suffix);
      break;
    }
    case TYPE_AP_OBJECT:
      fprintf(docs, "%s %s_ud%s", prefix, type.data.ud.sanitized_name, suffix);
      break;
    case TYPE_NONE:
    case TYPE_LITERAL:
      break;
  }
}

void emit_docs_param_type(struct type type, const char *prefix, const char *suffix) {
  if (type.type == TYPE_UINT32_T) {
    // we will try and convert int and numbers to uint32
    fprintf(docs, "%s uint32_t_ud|integer|number%s", prefix, suffix);
    return;
  }

  emit_docs_type(type, prefix, suffix);
}

void emit_docs_return_type(struct type type, int nullable) {
  // AP_Objects can be nil
  nullable |= (type.type == TYPE_AP_OBJECT);
  emit_docs_type(type, "---@return", (nullable == 0) ? "\n" : "|nil\n");
}

void emit_docs_method(const char *name, const char *method_name, struct method *method) {

  fprintf(docs, "-- desc\n");

  if (method->deprecate != NULL) {
    fprintf(docs, "---@deprecated %s\n", method->deprecate);
  }

  struct argument *arg = method->arguments;
  int count = 1;
  // input arguments
  while (arg != NULL) {
    if ((arg->type.type != TYPE_LITERAL) && (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) == 0) {
      char *param_name = (char *)allocate(20);
      sprintf(param_name, "---@param param%i", count);
      emit_docs_param_type(arg->type, param_name, "\n");
      free(param_name);
      count++;
    }
    arg = arg->next;
  }

  // return type
  if ((method->flags & TYPE_FLAGS_NULLABLE) == 0) {
    emit_docs_return_type(method->return_type, FALSE);
  }

  arg = method->arguments;
  // nullable and references returns
  while (arg != NULL) {
    if ((arg->type.type != TYPE_LITERAL) && (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE))) {
      emit_docs_return_type(arg->type, arg->type.flags & TYPE_FLAGS_NULLABLE);
    }
    arg = arg->next;
  }

  // function name
  fprintf(docs, "function %s:%s(", name, method_name);
  for (int i = 1; i < count; ++i) {
    fprintf(docs, "param%i", i);
    if (i < count-1) {
      fprintf(docs, ", ");
    }
  }
  fprintf(docs, ") end\n\n");
}

void emit_docs(struct userdata *node, int is_userdata, int emit_creation) {
  while(node) {
    char *name = (char *)allocate(strlen(node->rename ? node->rename : node->sanitized_name) + 5);
    if (is_userdata) {
      sprintf(name, "%s_ud", node->rename ? node->rename : node->sanitized_name);
    } else {
      sprintf(name, "%s", node->rename ? node->rename : node->sanitized_name);
    }


    fprintf(docs, "-- desc\n");
    if (is_userdata) {
      fprintf(docs, "---@class %s\n", name);
    }

    // enums
    if (node->enums != NULL) {
      struct userdata_enum *ud_enum = node->enums;
      while (ud_enum != NULL) {
        fprintf(docs, "---@field %s number\n", ud_enum->name);
        ud_enum = ud_enum->next;
      }
    }

    if (is_userdata) {
      // local userdata
      fprintf(docs, "local %s = {}\n\n", name);

      if (emit_creation && should_emit_creation(node)) {
        // creation function
        if (node->creation != NULL) {
          for (int i = 0; i < node->creation_args; ++i) {
            fprintf(docs, "---@param param%i UNKNOWN\n", i+1);
          }
        }

        fprintf(docs, "---@return %s\n", name);
        fprintf(docs, "function %s(", node->rename ? node->rename : node->sanitized_name);
        if (node->creation == NULL) {
          fprintf(docs, ") end\n\n");
        } else {
          for (int i = 0; i < node->creation_args; ++i) {
            fprintf(docs, "param%i", i+1);
            if (i < node->creation_args-1) {
              fprintf(docs, ", ");
            }
          }
          fprintf(docs, ") end\n\n");
        }
      }
    } else {
      // global
      fprintf(docs, "%s = {}\n\n", name);
    }


    // fields
    if (node->fields != NULL) {
      struct userdata_field *field = node->fields;
      while(field) {
          int valid_mask = (field->type.valid_mask.name != NULL) && (field->type.valid_mask.value != NULL);
          const char * return_postfix = valid_mask ? "|nil\n" : "\n";

          if (field->array_len == NULL) {
            // single value field
            if (field->access_flags & ACCESS_FLAG_READ) {
              fprintf(docs, "-- get field\n");
              emit_docs_type(field->type, "---@return", return_postfix);
              fprintf(docs, "function %s:%s() end\n\n", name, field->rename ? field->rename : field->name);
            }
            if (field->access_flags & ACCESS_FLAG_WRITE) {
              fprintf(docs, "-- set field\n");
              emit_docs_param_type(field->type, "---@param value", "\n");
              fprintf(docs, "function %s:%s(value) end\n\n", name, field->rename ? field->rename : field->name);
            }
          } else {
            // array field
            if (field->access_flags & ACCESS_FLAG_READ) {
              fprintf(docs, "-- get array field\n");
              fprintf(docs, "---@param index integer\n");
              emit_docs_type(field->type, "---@return", return_postfix);
              fprintf(docs, "function %s:%s(index) end\n\n", name, field->rename ? field->rename : field->name);
            }
            if (field->access_flags & ACCESS_FLAG_WRITE) {
              fprintf(docs, "-- set array field\n");
              fprintf(docs, "---@param index integer\n");
              emit_docs_param_type(field->type, "---@param value", "\n");
              fprintf(docs, "function %s:%s(index, value) end\n\n", name, field->rename ? field->rename : field->name);
            }
          }
        field = field->next;
      }
    }

    // methods
    struct method *method = node->methods;
    while(method) {
      emit_docs_method(name, method->rename ? method->rename : method->name, method);

      method = method->next;
    }

    // aliases
    struct method_alias *alias = node->method_aliases;
    while(alias) {
      if (alias->type == ALIAS_TYPE_NONE) {
        // find the method this is a alias of
        struct method * method = node->methods;
        while (method != NULL && strcmp(method->name, alias->name)) {
          method = method-> next;
        }
        if (method == NULL) {
          error(ERROR_DOCS, "Could not fine Method %s to alias to %s", alias->name, alias->alias);
        }

        emit_docs_method(name, alias->alias, method);

      } else if (alias->type == ALIAS_TYPE_MANUAL) {
          // Cant do a great job, don't know types or return
          fprintf(docs, "-- desc\n");

          for (int i = 0; i < alias->num_args; ++i) {
            fprintf(docs, "---@param param%i UNKNOWN\n", i+1);
          }

          for (int i = 0; i < alias->num_ret; ++i) {
            fprintf(docs, "---@return UNKNOWN\n");
          }

          fprintf(docs, "function %s:%s(", name, alias->alias);
          for (int i = 0; i < alias->num_args; ++i) {
            fprintf(docs, "param%i", i+1);
            if (i < alias->num_args-1) {
              fprintf(docs, ", ");
            }
          }
          fprintf(docs, ") end\n\n");
      }
      alias = alias->next;
    }
    fprintf(docs, "\n");
    free(name);
    node = node->next;
  }
}

// ---------------------------------------------------------------------------
// Remote mode (-r): Lua on the Cortex-M4 of CubeOrangePlus-M4Lua. The M4 gets
// the binding tables (<prefix>_tables.cpp) and the local boxed numerics
// (<prefix>_local.cpp); <prefix>_contract.h holds what both images share. The M7
// gets the dispatch (<prefix>_dispatch.cpp): the in-process binding functions
// and the blobs' wire codecs, compiled against AP_ScriptingM4_Remote.h.
// Method ids: FIRST_BINDING_METHOD + the stats id for methods, aliases and
// manual functions, then fields, operators and constructors. Ids cover the
// whole description; what a board compiled in is runtime data (m4:bindings).
// ---------------------------------------------------------------------------

#define REMOTE_FIRST_METHOD 64        // ScriptingM4::FIRST_BINDING_METHOD
#define REMOTE_MAX_VALUES 16          // ScriptingM4::MAX_VALUES
#define REMOTE_MAX_LABEL 63           // ScriptingM4::StatsMsg::name holds 64 bytes
#define REMOTE_MAX_TYPES 255
#define REMOTE_MAX_ROWS 4096
#define REMOTE_MAX_METHODS 4096
#define REMOTE_MAX_ENUMS 512

enum remote_kind {                    // ScriptingM4::BindingKind
  REMOTE_SINGLETON = 0,
  REMOTE_BLOB      = 1,
  REMOTE_HANDLE    = 2,
  REMOTE_LOCAL     = 3,
  REMOTE_GLOBAL    = 4,
  REMOTE_LOCAL_BLOB = 5,
  REMOTE_OWNED      = 6,
};

// ScriptingM4::ROW_*
#define ROW_HAS_SELF    0x01
#define ROW_WRITES_SELF 0x02
#define ROW_OPERATOR    0x04
#define ROW_CREATION    0x08
// ScriptingM4Remote::METHOD_*; the quota class (ScriptingM4::QuotaClass) is the high nibble
#define METHOD_WRITES_SELF 0x01
#define METHOD_QUOTA_FALSE 0x02

enum remote_quota {                   // ScriptingM4::QuotaClass
  QUOTA_NONE    = 0,
  QUOTA_TEXT    = 1,
  QUOTA_STORAGE = 2,
  QUOTA_LED     = 3,
  QUOTA_FILE_LOG = 4,
};

// userdata the M4 holds as wire bytes: plain data, no pointers, every member that
// matters described (or listed in remote_hidden_members)
static const char *remote_blob_userdata[] = {
  "AP_Camera::camera_state_t", "AP_ESC_Telem_Backend::TelemetryData", "AP_Filesystem::stat_t",
  "AP_HAL::CANFrame", "AP_MotorsMatrix_Scripting_Dynamic::factor_table", "AP_Servo_Telem::TelemetryData",
  "BattMonitorScript_State", "Cylinder_Status", "EFI_State", "RangeFinder::RangeFinder_State",
  "mavlink_camera_information_t", "mavlink_mission_item_int_t", "mavlink_video_stream_information_t",
  NULL };
// userdata the M4 implements itself (<prefix>_local.cpp, m4/m4_boxed_numerics.cpp)
static const char *remote_local_userdata[] = { "uint32_t", "uint64_t", NULL };
// pure-data userdata the M4 implements itself (AP_Math and AP_Common/Location built into the
// M4 image, m4/local): their fields, operators, constructor and the methods in
// remote_local_methods run on the M4; they cross to and from remote calls in the blob wire form
static const char *remote_local_blob_userdata[] = { "Vector2f", "Vector3f", "Quaternion", "Location", NULL };
// "Type:method" of a local blob type the M4 runs. Any other method of the type stays a remote
// row, since it may read vehicle state (Location:change_alt_frame reads home, origin and
// terrain): a method new to the description is remote until it is listed here
static const char *remote_local_methods[] = {
  "Vector2f:length", "Vector2f:normalize", "Vector2f:is_nan", "Vector2f:is_inf", "Vector2f:is_zero",
  "Vector2f:angle", "Vector2f:rotate", "Vector2f:copy",
  "Vector3f:length", "Vector3f:normalize", "Vector3f:is_nan", "Vector3f:is_inf", "Vector3f:is_zero",
  "Vector3f:dot", "Vector3f:cross", "Vector3f:scale", "Vector3f:copy", "Vector3f:xy", "Vector3f:rotate_xy",
  "Vector3f:angle",
  "Quaternion:length", "Quaternion:normalize", "Quaternion:get_euler_roll", "Quaternion:get_euler_pitch",
  "Quaternion:get_euler_yaw", "Quaternion:from_euler", "Quaternion:inverse", "Quaternion:earth_to_body",
  "Quaternion:to_axis_angle", "Quaternion:from_axis_angle", "Quaternion:from_angular_velocity",
  "Location:get_distance", "Location:offset", "Location:offset_bearing", "Location:offset_bearing_and_pitch",
  "Location:get_bearing", "Location:get_distance_NED", "Location:get_distance_NE", "Location:get_alt_frame",
  "Location:copy", "Location:set_alt_m",
  NULL };
// ap_objects the M7 keeps for its whole run: their handles need no release
static const char *remote_handle_ap_objects[] = {
  "AP_Proximity_Backend", "AP_RangeFinder_Backend", "AP_Airspeed_Backend",
  "AP_Vehicle::custom_mode_state", "RC_Channel", "AP_HAL::AnalogSource", "AP_EFI_Backend",
  NULL };
// userdata carrying M7 pointers or state: the object lives in the M7's object pool
// (AP_ScriptingM4_Objects.h), the M4 userdata holds an owned handle that its __gc releases
static const char *remote_owned_userdata[] = { "Parameter", NULL };
// an owned type's release when it has one of its own (its in-process __gc), else its destructor
struct remote_release {
  const char *type;
  const char *function;   // void function(void *object), declared in AP_ScriptingM4_Remote.h
};
static const struct remote_release remote_owned_release[] = { { NULL, NULL } };
// userdata carrying M7 pointers or state and the manual-only ap_objects: left out until
// IO and Manual Bindings. A userdata or ap_object in none of these lists fails generation
static const char *remote_deferred_userdata[] = {
  "AP_Scripting_SerialAccess", "CRSFMenu", "CRSFParameter", "DroneCAN_Handle",
  "SocketAPM", "ScriptingCANBuffer", "AP_HAL::I2CDevice", "AP_HAL::PWMSource",
  NULL };
// manual functions ported to the M7 adapter (AP_ScriptingM4_Manual.cpp)
static const char *remote_ready_manual[] = { "lua_millis", "lua_micros", "lua_new_Parameter", NULL };
// methods whose M7 function is a port of its own (AP_ScriptingM4_Manual*.cpp) in place of the
// generated one: what an M4 script must not make the M7 do that the generated code does
struct remote_replacement {
  const char *type;       // the description's name of the type
  const char *method;
  const char *function;   // declared in AP_ScriptingM4_Remote.h
};
static const struct remote_replacement remote_replaced_methods[] = {
  { "Parameter", "init_by_info", "lua_m4_Parameter_init_by_info" },   // an M7 allocation per call, never freed
  { NULL, NULL, NULL } };
static int remote_replacement_matched[sizeof(remote_replaced_methods) / sizeof(remote_replaced_methods[0])];

// members a blob carries besides its described fields: undescribed state a value the
// M7 made must keep when it comes back (mission items), and the valid_mask members
struct remote_hidden {
  const char *type;
  const char *member;
  enum field_type kind;
  const char *enum_type;     // a uint8_t enum class: the codec casts and checks 0..enum_max
  const char *enum_max;
};
static const struct remote_hidden remote_hidden_members[] = {
  { "Location", "loiter_ccw", TYPE_BOOLEAN, NULL, NULL },
  { "AP_Servo_Telem::TelemetryData", "present_types", TYPE_UINT16_T, NULL, NULL },
  { "mavlink_mission_item_int_t", "target_system", TYPE_UINT8_T, NULL, NULL },
  { "mavlink_mission_item_int_t", "target_component", TYPE_UINT8_T, NULL, NULL },
  { "mavlink_mission_item_int_t", "autocontinue", TYPE_UINT8_T, NULL, NULL },
  { "mavlink_mission_item_int_t", "mission_type", TYPE_UINT8_T, NULL, NULL },
  { "EFI_State", "engine_state", TYPE_UINT8_T, "Engine_State", "Engine_State::FAULT" },
  { "EFI_State", "crankshaft_sensor_status", TYPE_UINT8_T, "Crankshaft_Sensor_Status", "Crankshaft_Sensor_Status::ERROR" },
  { "EFI_State", "temperature_status", TYPE_UINT8_T, "Temperature_Status", "Temperature_Status::EGT_ABOVE_NOMINAL" },
  { "EFI_State", "oil_pressure_status", TYPE_UINT8_T, "Oil_Pressure_Status", "Oil_Pressure_Status::ABOVE_NOMINAL" },
  { "EFI_State", "detonation_status", TYPE_UINT8_T, "Detonation_Status", "Detonation_Status::OBSERVED" },
  { "EFI_State", "misfire_status", TYPE_UINT8_T, "Misfire_Status", "Misfire_Status::OBSERVED" },
  { "EFI_State", "debris_status", TYPE_UINT8_T, "Debris_Status", "Debris_Status::DETECTED" },
  { "EFI_State", "spark_plug_usage", TYPE_UINT8_T, "Spark_Plug_Usage", "Spark_Plug_Usage::BOTH_ACTIVE" },
  { NULL, NULL, TYPE_NONE, NULL, NULL } };

static int remote_hidden_member(const char *type, const char *member) {
  for (const struct remote_hidden *hidden = remote_hidden_members; hidden->type != NULL; hidden++) {
    if ((strcmp(hidden->type, type) == 0) && (strcmp(hidden->member, member) == 0)) {
      return TRUE;
    }
  }
  return FALSE;
}

// spec §7: per-method rate limits on expensive M7 work
struct remote_quota_entry {
  const char *type;
  const char *method;
  enum remote_quota quota;
};
static const struct remote_quota_entry remote_quotas[] = {
  { "GCS", "send_text", QUOTA_TEXT },
  { "GCS", "send_named_float", QUOTA_TEXT },
  { "GCS", "send_named_int", QUOTA_TEXT },
  { "GCS", "send_named_string", QUOTA_TEXT },
  { "AP_Param", "set_and_save_by_name", QUOTA_STORAGE },
  { "Parameter", "set_and_save", QUOTA_STORAGE },
  { "AP_Mission", "set_item", QUOTA_STORAGE },
  { "AP_Mission", "clear", QUOTA_STORAGE },
  { "AP_SerialLED", "send", QUOTA_LED },
  { "AP_Logger", "log_file_content", QUOTA_FILE_LOG },
  { NULL, NULL, QUOTA_NONE } };
static int remote_quota_matched[sizeof(remote_quotas) / sizeof(remote_quotas[0])];

struct remote_type {
  struct userdata *node;
  enum remote_kind kind;
  const char *lua_name;
  int first_row;
  int row_count;
  int first_enum;
  int enum_count;
};
struct remote_row {
  const char *lua_name;
  int method;
  int flags;
  int type;
};
struct remote_method {
  int used;
  int type;                  // owning type id
  char *function;            // the C function the M7 dispatch calls
  const char *dependency;    // besides the type's
  int flags;                 // METHOD_* | quota << 4
  int row;                   // the row naming it in SRPC
};
struct remote_enum {
  int type;
  const char *name;
};

static struct remote_type remote_types[REMOTE_MAX_TYPES];
static int remote_type_count;
static struct remote_row remote_rows[REMOTE_MAX_ROWS];
static int remote_row_count;
static struct remote_method remote_methods[REMOTE_MAX_METHODS];
static int remote_method_count;
static struct remote_enum remote_enums[REMOTE_MAX_ENUMS];
static int remote_enum_count;
static int remote_next_id;
static uint32_t remote_hash = 2166136261U;   // FNV-1a

static void remote_hash_byte(uint8_t byte) {
  remote_hash = (remote_hash ^ byte) * 16777619U;
}

static void remote_hash_int(int value) {
  for (int i = 0; i < 4; i++) {
    remote_hash_byte((uint8_t)((uint32_t)value >> (8 * i)));
  }
}

static void remote_hash_string(const char *text) {
  while (*text) {
    remote_hash_byte((uint8_t)*text++);
  }
  remote_hash_byte(0);
}

static void remote_hash_type(const struct type *t) {
  remote_hash_int(t->type);
  remote_hash_int((int)t->flags);
  if ((t->type == TYPE_USERDATA) || (t->type == TYPE_AP_OBJECT)) {
    remote_hash_string(t->data.ud.name);
  }
}

static int remote_in_list(const char *const *list, const char *name) {
  for (; *list != NULL; list++) {
    if (strcmp(*list, name) == 0) {
      return TRUE;
    }
  }
  return FALSE;
}

static const char *remote_owned_release_of(const char *type) {
  for (const struct remote_release *release = remote_owned_release; release->type != NULL; release++) {
    if (strcmp(release->type, type) == 0) {
      return release->function;
    }
  }
  return NULL;
}

// the M7 function replacing a generated method, or NULL
static const char *remote_replacement_of(const struct userdata *node, const struct method *method) {
  for (int i = 0; remote_replaced_methods[i].type != NULL; i++) {
    if ((strcmp(remote_replaced_methods[i].type, node->name) == 0) &&
        (strcmp(remote_replaced_methods[i].method, method->name) == 0)) {
      remote_replacement_matched[i] = TRUE;
      return remote_replaced_methods[i].function;
    }
  }
  return NULL;
}

// a userdata the wire carries as a blob: held as M7 bytes on the M4, or local there
static int remote_is_wire_userdata(const char *name) {
  return remote_in_list(remote_blob_userdata, name) || remote_in_list(remote_local_blob_userdata, name);
}

static int remote_kind_is_wire(enum remote_kind kind) {
  return (kind == REMOTE_BLOB) || (kind == REMOTE_LOCAL_BLOB);
}

static char *remote_format(const char *fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  const int length = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  if ((length < 0) || ((size_t)length >= sizeof(buffer))) {
    error(ERROR_INTERNAL, "remote: name too long: %s", fmt);
  }
  char *text = NULL;
  string_copy(&text, buffer);
  return text;
}

// a C++ function name as the M7 dispatch names it: DroneCAN_Handle::broadcast -> DroneCAN_Handle__broadcast
static char *remote_sanitize(const char *name) {
  char *out = remote_format("%s", name);
  for (char *c = out; *c != 0; c++) {
    if (*c == ':') {
      *c = '_';
    }
  }
  return out;
}

// the row flags of a ported manual function
static int remote_manual_flags(enum remote_kind kind, const struct method_alias *alias) {
  (void)alias;
  return (kind == REMOTE_SINGLETON) ? ROW_HAS_SELF : 0;
}

// a Rotation argument may name ROTATION_CUSTOM_1/2, which only the M7 applies: on the M4,
// AP::custom_rotations() is a stub that leaves the value unchanged (m4/shim/AP_CustomRotations)
static int remote_method_takes_rotation(const struct method *method) {
  for (const struct argument *arg = method->arguments; arg != NULL; arg = arg->next) {
    if ((arg->type.type == TYPE_ENUM) && (strcmp(arg->type.data.enum_name, "Rotation") == 0)) {
      return TRUE;
    }
  }
  return FALSE;
}

// a method of a local blob type that the M4 runs: it has no remote row or id
static int remote_method_is_local(const struct userdata *node, const struct method *method) {
  if (!remote_in_list(remote_local_blob_userdata, node->name)) {
    return FALSE;
  }
  const char *label = remote_format("%s:%s", node->name, method->name);
  if (!remote_in_list(remote_local_methods, label)) {
    return FALSE;
  }
  if (method->dependency != NULL) {
    // the M4 compiles every local type; a board's depends is runtime data, and only rows carry it
    error(ERROR_GENERAL, "remote: local method %s has a depends: keep it remote", label);
  }
  if (remote_method_takes_rotation(method)) {
    // stays a remote row: the M7 applies a custom rotation
    return FALSE;
  }
  return TRUE;
}

// every listed local method is a method of a local blob type: a typo would leave it remote unnoticed
static void remote_check_local_methods(void) {
  for (const char *const *entry = remote_local_methods; *entry != NULL; entry++) {
    int found = FALSE;
    for (const struct userdata *node = parsed_userdata; node != NULL; node = node->next) {
      if (!remote_in_list(remote_local_blob_userdata, node->name)) {
        continue;
      }
      for (const struct method *method = node->methods; method != NULL; method = method->next) {
        found |= strcmp(remote_format("%s:%s", node->name, method->name), *entry) == 0;
      }
    }
    if (!found) {
      error(ERROR_GENERAL, "remote: local method %s is no method of a local blob type", *entry);
    }
  }
}

// a type a remote call can carry
static int remote_type_supported(const struct type *t) {
  switch (t->type) {
    case TYPE_BOOLEAN:
    case TYPE_FLOAT:
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_UINT32_T:
    case TYPE_NONE:
    case TYPE_STRING:
    case TYPE_ENUM:
    case TYPE_LITERAL:
      return TRUE;
    case TYPE_USERDATA:
      return remote_is_wire_userdata(t->data.ud.name) || remote_in_list(remote_local_userdata, t->data.ud.name);
    case TYPE_AP_OBJECT:
      return remote_in_list(remote_handle_ap_objects, t->data.ud.name);
  }
  error(ERROR_INTERNAL, "remote: unknown type kind %d", (int)t->type);
  return FALSE;
}

static int remote_method_supported(const struct method *method) {
  if (!remote_type_supported(&method->return_type)) {
    return FALSE;
  }
  int args = 1;      // the self
  int results = 1;   // the return value
  for (const struct argument *arg = method->arguments; arg != NULL; arg = arg->next) {
    if (!remote_type_supported(&arg->type)) {
      return FALSE;
    }
    if (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERENCE)) {
      results++;
    } else if (arg->type.type != TYPE_LITERAL) {
      args++;
    }
  }
  // one value stays free for the write-back of a blob self
  if ((args > REMOTE_MAX_VALUES) || (results > REMOTE_MAX_VALUES - 1)) {
    error(ERROR_GENERAL, "remote: %s carries more values than a frame holds", method->name);
  }
  return TRUE;
}

static int remote_kind_of(const struct userdata *node) {
  switch (node->ud_type) {
    case UD_SINGLETON:
      return REMOTE_SINGLETON;
    case UD_USERDATA:
      if (remote_in_list(remote_local_userdata, node->name)) {
        return REMOTE_LOCAL;
      }
      if (remote_in_list(remote_local_blob_userdata, node->name)) {
        return REMOTE_LOCAL_BLOB;
      }
      if (remote_in_list(remote_owned_userdata, node->name)) {
        return REMOTE_OWNED;
      }
      if (remote_in_list(remote_blob_userdata, node->name)) {
        return REMOTE_BLOB;
      }
      break;
    case UD_AP_OBJECT:
      if (remote_in_list(remote_handle_ap_objects, node->name)) {
        return REMOTE_HANDLE;
      }
      break;
    case UD_GLOBAL:
      return REMOTE_GLOBAL;
    default:
      error(ERROR_INTERNAL, "remote: %s has an unknown userdata kind %d", node->name, (int)node->ud_type);
  }
  if (remote_in_list(remote_deferred_userdata, node->name)) {
    return -1;
  }
  error(ERROR_GENERAL, "remote: %s is in no remote list (blob, local, handle or deferred)", node->name);
  return -1;
}

// a name in a remote list that the description does not declare is a typo
static void remote_check_declared(const char *const *list) {
  for (; *list != NULL; list++) {
    int found = FALSE;
    for (const struct userdata *node = parsed_userdata; node != NULL; node = node->next) {
      found |= strcmp(node->name, *list) == 0;
    }
    for (const struct userdata *node = parsed_ap_objects; node != NULL; node = node->next) {
      found |= strcmp(node->name, *list) == 0;
    }
    if (!found) {
      error(ERROR_GENERAL, "remote: %s is in a remote list but not in the description", *list);
    }
  }
}

static int remote_add_row(const char *lua_name, int method, int flags, int type) {
  if (remote_row_count >= REMOTE_MAX_ROWS) {
    error(ERROR_INTERNAL, "remote: too many rows");
  }
  const char *type_name = remote_types[type - 1].lua_name;
  if (strlen(type_name) + 1 + strlen(lua_name) > REMOTE_MAX_LABEL) {
    error(ERROR_GENERAL, "remote: %s:%s is longer than an SRPC name", type_name, lua_name);
  }
  struct remote_row *row = &remote_rows[remote_row_count];
  row->lua_name = lua_name;
  row->method = method;
  row->flags = flags;
  row->type = type;
  remote_hash_string(lua_name);
  remote_hash_int(method);
  remote_hash_int(flags);
  return remote_row_count++;
}

static void remote_add_method(int id, int type, char *function, const char *dependency, int flags, int row) {
  const int index = id - REMOTE_FIRST_METHOD;
  if ((index < 0) || (index >= REMOTE_MAX_METHODS)) {
    error(ERROR_INTERNAL, "remote: method id %d out of range", id);
  }
  struct remote_method *method = &remote_methods[index];
  if (method->used) {
    if (strcmp(method->function, function) != 0) {
      error(ERROR_INTERNAL, "remote: id %d is both %s and %s", id, method->function, function);
    }
    return;
  }
  method->used = TRUE;
  method->type = type;
  method->function = function;
  method->dependency = dependency;
  method->flags = flags;
  method->row = row;
  if (index + 1 > remote_method_count) {
    remote_method_count = index + 1;
  }
}

static int remote_quota_flags(const struct userdata *node, const struct method *method) {
  for (int i = 0; remote_quotas[i].type != NULL; i++) {
    const struct remote_quota_entry *quota = &remote_quotas[i];
    if ((strcmp(quota->type, node->name) != 0) || (strcmp(quota->method, method->name) != 0)) {
      continue;
    }
    remote_quota_matched[i] = TRUE;
    if (method->return_type.type == TYPE_NONE) {
      return quota->quota << 4;                        // over its quota: dropped
    }
    if ((method->return_type.type == TYPE_BOOLEAN) && !(method->flags & TYPE_FLAGS_NULLABLE)) {
      return (quota->quota << 4) | METHOD_QUOTA_FALSE; // over its quota: false
    }
    error(ERROR_GENERAL, "remote: a quota on %s:%s needs a void or boolean method", node->name, method->name);
  }
  return 0;
}

static void remote_collect_type(struct userdata *node, enum remote_kind kind) {
  if (remote_type_count >= REMOTE_MAX_TYPES) {
    error(ERROR_INTERNAL, "remote: too many types");
  }
  const int type_id = ++remote_type_count;
  struct remote_type *rt = &remote_types[type_id - 1];
  rt->node = node;
  rt->kind = kind;
  if (kind == REMOTE_GLOBAL) {
    rt->lua_name = "global";
  } else if (kind == REMOTE_HANDLE) {
    rt->lua_name = node->name;   // new_ap_object() names the metatable so
  } else {
    rt->lua_name = node->rename ? node->rename : node->name;
  }
  for (int i = 0; i < type_id - 1; i++) {
    if (strcmp(remote_types[i].lua_name, rt->lua_name) == 0) {
      error(ERROR_GENERAL, "remote: two types named %s", rt->lua_name);
    }
  }
  rt->first_row = remote_row_count;
  rt->first_enum = remote_enum_count;
  remote_hash_int(kind);
  remote_hash_string(rt->lua_name);

  if (kind != REMOTE_LOCAL) {
    const int wire = remote_kind_is_wire(kind);
    const int self_flags = (kind == REMOTE_SINGLETON) ? ROW_HAS_SELF : (wire ? ROW_WRITES_SELF : 0);
    const int method_flags = wire ? METHOD_WRITES_SELF : 0;
    for (struct method *method = node->methods; method != NULL; method = method->next) {
      if (!remote_method_supported(method)) {
        continue;
      }
      if (remote_method_is_local(node, method)) {
        // no row and no id; hashed, so an M4 image that runs it locally never meets an M7 that serves it
        remote_hash_string("local");
        remote_hash_string(method->name);
        remote_hash_type(&method->return_type);
        for (const struct argument *arg = method->arguments; arg != NULL; arg = arg->next) {
          remote_hash_type(&arg->type);
        }
        continue;
      }
      const int id = REMOTE_FIRST_METHOD + method->stats_id;
      const int row = remote_add_row(method->rename ? method->rename : method->name, id, self_flags, type_id);
      const char *replacement = remote_replacement_of(node, method);
      remote_add_method(id, type_id,
                        replacement != NULL ? remote_format("%s", replacement) :
                                              remote_format("%s_%s", node->sanitized_name, method->sanitized_name),
                        method->dependency, method_flags | remote_quota_flags(node, method), row);
      if (replacement != NULL) {
        remote_hash_string(replacement);   // an image whose M7 runs another function is another contract
      }
      remote_hash_type(&method->return_type);
      for (const struct argument *arg = method->arguments; arg != NULL; arg = arg->next) {
        remote_hash_type(&arg->type);
      }
    }
    for (struct method_alias *alias = node->method_aliases; alias != NULL; alias = alias->next) {
      if (kind == REMOTE_LOCAL_BLOB) {
        error(ERROR_GENERAL, "remote: local type %s has the alias %s: local types take none", node->name, alias->alias);
      }
      const int id = REMOTE_FIRST_METHOD + alias->stats_id;
      if (alias->type == ALIAS_TYPE_NONE) {
        if (remote_methods[id - REMOTE_FIRST_METHOD].used) {
          remote_add_row(alias->alias, id, self_flags, type_id);   // its target's id and function
        }
      } else if ((alias->type == ALIAS_TYPE_MANUAL) && remote_in_list(remote_ready_manual, alias->name)) {
        const int row = remote_add_row(alias->alias, id, remote_manual_flags(kind, alias), type_id);
        remote_add_method(id, type_id, remote_sanitize(alias->name), alias->dependency, 0, row);
      }
    }
    for (struct userdata_field *field = node->fields; field != NULL; field = field->next) {
      if (!remote_type_supported(&field->type)) {
        if (wire) {
          // the member would be reset on every round trip
          error(ERROR_GENERAL, "remote: blob %s.%s has a type the wire cannot carry", node->name, field->name);
        }
        continue;
      }
      // the setter sets the mask bit: the mask has to cross with the blob
      const char *valid_mask = field->type.valid_mask.name;
      if (wire && (valid_mask != NULL) && !remote_hidden_member(node->name, valid_mask)) {
        error(ERROR_GENERAL, "remote: %s.%s has a valid_mask: give the mask a wire member first", node->name, field->name);
      }
      if (kind == REMOTE_LOCAL_BLOB) {
        if (field->dependency != NULL) {
          error(ERROR_GENERAL, "remote: local field %s.%s has a depends: the M4 cannot evaluate it", node->name, field->name);
        }
        if (field->type.type == TYPE_USERDATA) {
          error(ERROR_GENERAL, "remote: local field %s.%s holds a userdata: the M4 has no codec for it", node->name, field->name);
        }
      } else {
        const int id = REMOTE_FIRST_METHOD + remote_next_id++;
        const int row = remote_add_row(field->rename ? field->rename : field->name, id, self_flags, type_id);
        remote_add_method(id, type_id, remote_format("%s_%s", node->sanitized_name, field->name),
                          field->dependency, method_flags, row);
      }
      remote_hash_string(field->name);
      remote_hash_type(&field->type);
      remote_hash_int((int)field->access_flags);
      remote_hash_string(field->array_len ? field->array_len : "");
      remote_hash_string(valid_mask ? valid_mask : "");
    }
    if (wire) {
      for (uint32_t op = 1; op < OP_LAST; op <<= 1) {
        const char *op_name = get_name_for_operation((enum operator_type)(node->operations & op));
        if (op_name == NULL) {
          continue;
        }
        if (kind == REMOTE_LOCAL_BLOB) {
          remote_hash_string(op_name);
          continue;
        }
        const int id = REMOTE_FIRST_METHOD + remote_next_id++;
        const int row = remote_add_row(op_name, id, ROW_OPERATOR, type_id);
        remote_add_method(id, type_id, remote_format("%s_%s", node->sanitized_name, op_name), NULL, 0, row);
      }
      if ((node->creation == NULL) && should_emit_creation(node)) {
        if (kind == REMOTE_LOCAL_BLOB) {
          remote_hash_string("new");
        } else {
          // the global is the type's name; "type:new" keeps the SRPC name short
          const int id = REMOTE_FIRST_METHOD + remote_next_id++;
          const int row = remote_add_row("new", id, ROW_CREATION, type_id);
          remote_add_method(id, type_id, remote_format("lua_new_%s", node->sanitized_name), NULL, 0, row);
        }
      }
      for (const struct remote_hidden *hidden = remote_hidden_members; hidden->type != NULL; hidden++) {
        if (strcmp(hidden->type, node->name) == 0) {
          remote_hash_string(hidden->member);
          remote_hash_int(hidden->kind);
          if (hidden->enum_type != NULL) {
            remote_hash_string(hidden->enum_type);
            remote_hash_string(hidden->enum_max);
          }
        }
      }
    }
    if ((kind == REMOTE_OWNED) && (node->creation != NULL) && remote_in_list(remote_ready_manual, node->creation)) {
      // the constructor is the ported creation function; an owned type gets no generated one,
      // which would leave its M7 pointers unset (CRSFParameter)
      const int id = REMOTE_FIRST_METHOD + remote_next_id++;
      const int row = remote_add_row("new", id, ROW_CREATION, type_id);
      remote_add_method(id, type_id, remote_sanitize(node->creation), NULL, 0, row);
    }
  }
  if ((kind == REMOTE_LOCAL_BLOB) && (node->enums != NULL)) {
    error(ERROR_GENERAL, "remote: local type %s has enums: the M4 local index does not serve them", node->name);
  }
  for (struct userdata_enum *ud_enum = node->enums; ud_enum != NULL; ud_enum = ud_enum->next) {
    if (remote_enum_count >= REMOTE_MAX_ENUMS) {
      error(ERROR_INTERNAL, "remote: too many enums");
    }
    remote_enums[remote_enum_count].type = type_id;
    remote_enums[remote_enum_count].name = ud_enum->name;
    remote_enum_count++;
    remote_hash_string(ud_enum->name);
  }
  rt->row_count = remote_row_count - rt->first_row;
  rt->enum_count = remote_enum_count - rt->first_enum;
}

static int remote_type_id(const char *node_name) {
  for (int i = 0; i < remote_type_count; i++) {
    if ((remote_types[i].node != NULL) && (remote_types[i].node->name != NULL) &&
        (strcmp(remote_types[i].node->name, node_name) == 0)) {
      return i + 1;
    }
  }
  error(ERROR_GENERAL, "remote: no type %s", node_name);
  return 0;
}

// after the in-process emission: every stats id is assigned
static void remote_collect(void) {
  stats_ids_frozen = TRUE;
  remote_next_id = next_stats_id;
  for (struct userdata *node = parsed_singletons; node != NULL; node = node->next) {
    remote_collect_type(node, REMOTE_SINGLETON);
  }
  for (struct userdata *node = parsed_userdata; node != NULL; node = node->next) {
    const int kind = remote_kind_of(node);
    if (kind >= 0) {
      remote_collect_type(node, (enum remote_kind)kind);
    }
  }
  for (struct userdata *node = parsed_ap_objects; node != NULL; node = node->next) {
    const int kind = remote_kind_of(node);
    if (kind >= 0) {
      remote_collect_type(node, (enum remote_kind)kind);
    }
  }
  if (parsed_globals != NULL) {
    remote_collect_type(parsed_globals, REMOTE_GLOBAL);
  }
  remote_check_declared(remote_blob_userdata);
  remote_check_declared(remote_local_userdata);
  remote_check_declared(remote_handle_ap_objects);
  remote_check_declared(remote_deferred_userdata);
  remote_check_declared(remote_local_blob_userdata);
  remote_check_declared(remote_owned_userdata);
  for (const struct remote_release *release = remote_owned_release; release->type != NULL; release++) {
    if (!remote_in_list(remote_owned_userdata, release->type)) {
      error(ERROR_GENERAL, "remote: release %s of %s, which is no owned type", release->function, release->type);
    }
  }
  for (int i = 0; remote_replaced_methods[i].type != NULL; i++) {
    if (!remote_replacement_matched[i]) {
      error(ERROR_GENERAL, "remote: replacement %s of %s:%s matches no method", remote_replaced_methods[i].function,
            remote_replaced_methods[i].type, remote_replaced_methods[i].method);
    }
  }
  remote_check_local_methods();
  for (const struct remote_hidden *hidden = remote_hidden_members; hidden->type != NULL; hidden++) {
    if (!remote_kind_is_wire(remote_types[remote_type_id(hidden->type) - 1].kind)) {
      error(ERROR_GENERAL, "remote: hidden member %s.%s of a type that is no blob", hidden->type, hidden->member);
    }
  }
  for (int i = 0; remote_quotas[i].type != NULL; i++) {
    if (!remote_quota_matched[i]) {
      error(ERROR_GENERAL, "remote: quota on %s:%s matches no method", remote_quotas[i].type, remote_quotas[i].method);
    }
  }
  if ((remote_enum_count == 0) || (remote_method_count == 0)) {
    error(ERROR_GENERAL, "remote: empty binding table");
  }
}

static FILE *remote_open(const char *prefix, const char *suffix) {
  char *name = remote_format("%s_%s", prefix, suffix);
  FILE *f = fopen(name, "w");
  if (f == NULL) {
    error(ERROR_GENERAL, "Unable to open the output file: %s", name);
  }
  return f;
}

static void remote_emit_contract(FILE *f) {
  fprintf(f, "#pragma once\n// auto generated by gen-bindings -r, don't manually edit.  See README.md for details.\n\n");
  fprintf(f, "#include <stdint.h>\n\nnamespace ScriptingM4 {\n\n");
  fprintf(f, "// FNV-1a of the generated binding table: types, rows, ids, argument, return and field types\n");
  fprintf(f, "static const uint32_t BINDING_HASH = 0x%08xU;\n", (unsigned)remote_hash);
  fprintf(f, "// method ids FIRST_BINDING_METHOD .. FIRST_BINDING_METHOD + BINDING_METHOD_COUNT - 1\n");
  fprintf(f, "static const uint16_t BINDING_METHOD_COUNT = %d;\n", remote_method_count);
  fprintf(f, "// type ids 1 .. BINDING_TYPE_COUNT\n");
  fprintf(f, "static const uint8_t BINDING_TYPE_COUNT = %d;\n", remote_type_count);
  fprintf(f, "static const uint16_t BINDING_ROW_COUNT = %d;\n", remote_row_count);
  fprintf(f, "static const uint16_t BINDING_ENUM_COUNT = %d;\n", remote_enum_count);
  fprintf(f, "static const uint8_t BINDING_TYPE_UINT32 = %d;\n", remote_type_id("uint32_t"));
  fprintf(f, "static const uint8_t BINDING_TYPE_UINT64 = %d;\n", remote_type_id("uint64_t"));
  fprintf(f, "\n}  // namespace ScriptingM4\n");
}

static void remote_emit_tables(FILE *f) {
  fprintf(f, "// auto generated by gen-bindings -r, don't manually edit.  See README.md for details.\n\n");
  fprintf(f, "#include <AP_Scripting/AP_ScriptingM4_Bindings.h>\n\nnamespace ScriptingM4 {\n\n");
  fprintf(f, "const BindingType binding_types[BINDING_TYPE_COUNT] = {\n");
  for (int i = 0; i < remote_type_count; i++) {
    const struct remote_type *rt = &remote_types[i];
    fprintf(f, "    { \"%s\", %d, %d, %d, %d, %d },\n", rt->lua_name, rt->kind, rt->enum_count,
            rt->first_row, rt->row_count, rt->first_enum);
  }
  fprintf(f, "};\n\nconst BindingRow binding_rows[BINDING_ROW_COUNT] = {\n");
  for (int i = 0; i < remote_row_count; i++) {
    const struct remote_row *row = &remote_rows[i];
    fprintf(f, "    { \"%s\", %d, 0x%02x, %d },\n", row->lua_name, row->method, row->flags, row->type);
  }
  fprintf(f, "};\n\nconst char *const binding_enum_names[BINDING_ENUM_COUNT] = {\n");
  for (int i = 0; i < remote_enum_count; i++) {
    fprintf(f, "    \"%s\",\n", remote_enums[i].name);
  }
  fprintf(f, "};\n\nconst uint16_t binding_method_rows[BINDING_METHOD_COUNT] = {\n");
  for (int i = 0; i < remote_method_count; i++) {
    fprintf(f, "    %d,\n", remote_methods[i].used ? remote_methods[i].row : 0xFFFF);
  }
  fprintf(f, "};\n\n}  // namespace ScriptingM4\n");
}

static const char *remote_wire_ctype(const struct type *t) {
  switch (t->type) {
    case TYPE_BOOLEAN: return "uint8_t";
    case TYPE_FLOAT:   return "float";
    case TYPE_INT8_T:  return "int8_t";
    case TYPE_INT16_T: return "int16_t";
    case TYPE_INT32_T: return "int32_t";
    case TYPE_UINT8_T: return "uint8_t";
    case TYPE_UINT16_T: return "uint16_t";
    case TYPE_UINT32_T: return "uint32_t";
    case TYPE_ENUM:    return "int32_t";
    case TYPE_NONE:
    case TYPE_STRING:
    case TYPE_LITERAL:
    case TYPE_USERDATA:
    case TYPE_AP_OBJECT:
      break;
  }
  return NULL;
}

static int remote_codec_ranges = TRUE;   // FALSE for the M4: the M7's values are trusted

// one value of a blob's wire form: write it from, or read it into, member
static void remote_emit_wire_value(FILE *f, int write, const char *member, const struct type *t, const char *indent) {
  if (t->type == TYPE_USERDATA) {
    if (!remote_is_wire_userdata(t->data.ud.name)) {
      error(ERROR_GENERAL, "remote: blob member %s is a %s, which the M4 cannot hold", member, t->data.ud.name);
    }
    if (write) {
      fprintf(f, "%swrite_%s(w, &%s);\n", indent, t->data.ud.sanitized_name, member);
    } else {
      fprintf(f, "%sif (!read_%s(r, &%s)) {\n%s    return false;\n%s}\n", indent, t->data.ud.sanitized_name, member, indent, indent);
    }
    return;
  }
  const char *ctype = remote_wire_ctype(t);
  if (ctype == NULL) {
    error(ERROR_GENERAL, "remote: blob member %s has no wire form", member);
  }
  if (write) {
    if (t->type == TYPE_BOOLEAN) {
      fprintf(f, "%sw.write(uint8_t(%s ? 1 : 0));\n", indent, member);
    } else {
      fprintf(f, "%sw.write(%s(%s));\n", indent, ctype, member);
    }
    return;
  }
  fprintf(f, "%s{\n", indent);
  fprintf(f, "%s    %s v;\n", indent, ctype);
  fprintf(f, "%s    if (!r.read(v)) {\n%s        return false;\n%s    }\n", indent, indent, indent);
  if (t->type == TYPE_BOOLEAN) {
    fprintf(f, "%s    if (v > 1) {\n%s        return false;\n%s    }\n", indent, indent, indent);
    fprintf(f, "%s    %s = (v != 0);\n", indent, member);
  } else {
    // the range the field's setter checks: a blob carries nothing a setter would refuse
    if (remote_codec_ranges && (t->range != NULL) && !(t->flags & TYPE_FLAGS_NO_RANGE_CHECK)) {
      if (t->type == TYPE_FLOAT) {
        fprintf(f, "%s    if (!(v >= float(%s) && v <= float(%s))) {\n", indent, t->range->low, t->range->high);
      } else {
        // widened in a variable: a bound at the wire type's own limit is no -Wtype-limits error
        fprintf(f, "%s    const int64_t wide = v;\n", indent);
        fprintf(f, "%s    if (wide < int64_t(%s) || wide > int64_t(%s)) {\n", indent, t->range->low, t->range->high);
      }
      fprintf(f, "%s        return false;\n%s    }\n", indent, indent);
    }
    if (t->type == TYPE_ENUM) {
      fprintf(f, "%s    %s = static_cast<%s>(v);\n", indent, member, t->data.enum_name);
    } else {
      fprintf(f, "%s    %s = v;\n", indent, member);
    }
  }
  fprintf(f, "%s}\n", indent);
}

static void remote_emit_wire_member(FILE *f, int write, const char *name, const char *array_len, const struct type *t) {
  if (array_len != NULL) {
    fprintf(f, "    for (int i = 0; i < int(%s); i++) {\n", array_len);
    remote_emit_wire_value(f, write, remote_format("ud->%s[i]", name), t, "        ");
    fprintf(f, "    }\n");
  } else {
    remote_emit_wire_value(f, write, remote_format("ud->%s", name), t, "    ");
  }
}

// a hidden member travels as its declared kind; an enum member as uint8_t, checked 0..enum_max
static void remote_emit_hidden(FILE *f, int write, const struct remote_hidden *hidden) {
  if ((hidden->kind != TYPE_BOOLEAN) && (hidden->kind != TYPE_UINT8_T) && (hidden->kind != TYPE_UINT16_T)) {
    error(ERROR_GENERAL, "remote: hidden member %s.%s needs a bool, uint8_t or uint16_t kind", hidden->type, hidden->member);
  }
  struct type t;
  memset(&t, 0, sizeof(t));
  t.type = hidden->kind;
  const char *member = remote_format("ud->%s", hidden->member);
  if (write && (hidden->kind != TYPE_BOOLEAN)) {
    // the wire width is the member's (a hidden bool may be a bitfield: Location.loiter_ccw)
    fprintf(f, "    static_assert(sizeof(%s) == sizeof(%s), \"%s.%s has another width than its wire form\");\n",
            member, remote_wire_ctype(&t), hidden->type, hidden->member);
  }
  if (hidden->enum_type == NULL) {
    remote_emit_wire_value(f, write, member, &t, "    ");
    return;
  }
  if (hidden->kind != TYPE_UINT8_T) {
    error(ERROR_GENERAL, "remote: hidden enum member %s.%s needs the uint8_t kind", hidden->type, hidden->member);
  }
  if (write) {
    fprintf(f, "    w.write(uint8_t(%s));\n", member);
    return;
  }
  fprintf(f, "    {\n");
  fprintf(f, "        uint8_t v;\n");
  fprintf(f, "        if (!r.read(v) || v > uint8_t(%s)) {\n", hidden->enum_max);
  fprintf(f, "            return false;\n");
  fprintf(f, "        }\n");
  fprintf(f, "        %s = static_cast<%s>(v);\n", member, hidden->enum_type);
  fprintf(f, "    }\n");
}

// a described field as the wire carries it: the member's width (an enum fits int32), and an
// array's declared length is the member's, so M4 bytes are never decoded past it
static void remote_emit_field_asserts(FILE *f, const struct userdata *node, const struct userdata_field *field) {
  const char *member = remote_format("ud->%s", field->name);
  const char *element = field->array_len ? remote_format("ud->%s[0]", field->name) : member;
  if (field->array_len != NULL) {
    fprintf(f, "    static_assert(sizeof(%s) / sizeof(%s) == size_t(%s), \"%s.%s is described with another length\");\n",
            member, element, field->array_len, node->name, field->name);
  }
  const char *ctype = remote_wire_ctype(&field->type);
  switch (field->type.type) {
    case TYPE_BOOLEAN:
      // may be a bitfield (Location): sizeof cannot apply; the wire carries 0 or 1
      break;
    case TYPE_ENUM:
      fprintf(f, "    static_assert(sizeof(%s) <= sizeof(%s), \"%s.%s is wider than its wire form\");\n",
              element, ctype, node->name, field->name);
      break;
    case TYPE_USERDATA:
      fprintf(f, "    static_assert(std::is_same<std::remove_cv<std::remove_reference<decltype(%s)>::type>::type, %s>::value, \"%s.%s is described with another type\");\n",
              element, field->type.data.ud.name, node->name, field->name);
      break;
    default:
      if (ctype != NULL) {
        fprintf(f, "    static_assert(sizeof(%s) == sizeof(%s), \"%s.%s has another width than its wire form\");\n",
                element, ctype, node->name, field->name);
      }
      break;
  }
}

// the wire form's length on this board, by the same depends guards as the codec; a function of
// ud because array lengths are described through it (ARRAY_SIZE(ud->data)), only inside sizeof
static void remote_emit_wire_size(FILE *f, int type_index, int *emitted) {
  if (emitted[type_index]) {
    return;
  }
  emitted[type_index] = TRUE;
  const struct userdata *node = remote_types[type_index].node;
  // nested types first: their constant is part of this one
  for (const struct userdata_field *field = node->fields; field != NULL; field = field->next) {
    if (field->type.type == TYPE_USERDATA) {
      remote_emit_wire_size(f, remote_type_id(field->type.data.ud.name) - 1, emitted);
    }
  }
  start_dependency(f, node->dependency);
  fprintf(f, "static constexpr size_t wire_size_%s(__attribute__((unused)) const %s *ud)\n{\n    return 0\n",
          node->sanitized_name, node->name);
  for (const struct userdata_field *field = node->fields; field != NULL; field = field->next) {
    start_dependency(f, field->dependency);
    const char *term = (field->type.type == TYPE_USERDATA) ?
                       remote_format("wire_size_%s(nullptr)", field->type.data.ud.sanitized_name) :
                       remote_format("sizeof(%s)", remote_wire_ctype(&field->type));
    if (field->array_len != NULL) {
      fprintf(f, "    + %s * size_t(%s)\n", term, field->array_len);
    } else {
      fprintf(f, "    + %s\n", term);
    }
    end_dependency(f, field->dependency);
  }
  for (const struct remote_hidden *hidden = remote_hidden_members; hidden->type != NULL; hidden++) {
    if (strcmp(hidden->type, node->name) == 0) {
      struct type t;
      memset(&t, 0, sizeof(t));
      t.type = hidden->kind;
      fprintf(f, "    + sizeof(%s)\n", remote_wire_ctype(&t));
    }
  }
  fprintf(f, "    ;\n}\n");
  fprintf(f, "static_assert(wire_size_%s(nullptr) <= ScriptingM4::MAX_BLOB, \"%s's wire form is longer than a blob\");\n",
          node->sanitized_name, node->name);
  end_dependency(f, node->dependency);
}

// write_/read_/decode_/encode_<type>: the described fields in order, then the hidden members.
// The writer and the reader take the same depends guards, so a board's wire form is one length
static void remote_emit_codec(FILE *f, const struct remote_type *rt) {
  struct userdata *node = rt->node;
  start_dependency(f, node->dependency);
  fprintf(f, "static_assert(std::is_trivially_destructible<%s>::value, \"remote blobs are never destroyed\");\n\n", node->name);
  for (int write = 1; write >= 0; write--) {
    if (write) {
      fprintf(f, "static void write_%s(WireWriter &w, const %s *ud) {\n", node->sanitized_name, node->name);
    } else {
      fprintf(f, "static bool read_%s(WireReader &r, %s *ud) {\n", node->sanitized_name, node->name);
    }
    for (struct userdata_field *field = node->fields; field != NULL; field = field->next) {
      // the setter sets the mask bit: the mask crosses as a hidden member
      const char *valid_mask = field->type.valid_mask.name;
      if ((valid_mask != NULL) && !remote_hidden_member(node->name, valid_mask)) {
        error(ERROR_GENERAL, "remote: %s.%s has a valid_mask: give the mask a wire member first", node->name, field->name);
      }
      start_dependency(f, field->dependency);
      if (write) {
        remote_emit_field_asserts(f, node, field);
      }
      remote_emit_wire_member(f, write, field->name, field->array_len, &field->type);
      end_dependency(f, field->dependency);
    }
    for (const struct remote_hidden *hidden = remote_hidden_members; hidden->type != NULL; hidden++) {
      if (strcmp(hidden->type, node->name) == 0) {
        remote_emit_hidden(f, write, hidden);
      }
    }
    if (!write) {
      fprintf(f, "    return true;\n");
    }
    fprintf(f, "}\n\n");
  }
  fprintf(f, "static bool decode_%s(WireReader &r, void *storage) {\n", node->sanitized_name);
  fprintf(f, "    %s *ud = new (storage) %s();\n", node->name, node->name);
  fprintf(f, "    return read_%s(r, ud) && r.done();\n}\n\n", node->sanitized_name);
  fprintf(f, "static void encode_%s(WireWriter &w, const void *object) {\n", node->sanitized_name);
  fprintf(f, "    write_%s(w, static_cast<const %s *>(object));\n}\n\n", node->sanitized_name, node->name);
  end_dependency(f, node->dependency);
}

// the __index of a local blob type on the M4: its local methods and fields as in-process,
// then its remote rows (ScriptingM4::remote_row_index)
static void remote_emit_local_index(FILE *f, const struct remote_type *rt, int type_id) {
  const struct userdata *node = rt->node;
  fprintf(f, "const luaL_Reg %s_meta[] = {\n", node->sanitized_name);
  for (struct method *method = node->methods; method != NULL; method = method->next) {
    if (remote_method_is_local(node, method)) {
      fprintf(f, "    {\"%s\", %s_%s},\n", method->rename ? method->rename : method->name,
              node->sanitized_name, method->sanitized_name);
    }
  }
  for (struct userdata_field *field = node->fields; field != NULL; field = field->next) {
    fprintf(f, "    {\"%s\", %s_%s},\n", field->rename ? field->rename : field->name, node->sanitized_name, field->name);
  }
  fprintf(f, "};\n\n");
  if (node->operations) {
    fprintf(f, "const luaL_Reg %s_operators[] = {\n", node->sanitized_name);
    for (uint32_t op = 1; op < OP_LAST; op <<= 1) {
      const char *op_name = get_name_for_operation((enum operator_type)(node->operations & op));
      if (op_name != NULL) {
        fprintf(f, "    {\"%s\", %s_%s},\n", op_name, node->sanitized_name, op_name);
      }
    }
    fprintf(f, "    {NULL, NULL},\n};\n\n");
  }
  fprintf(f, "static int %s_index(lua_State *L) {\n", node->sanitized_name);
  if (rt->row_count > 0) {
    fprintf(f, "    return load_function(L, %s_meta, ARRAY_SIZE(%s_meta)) || ScriptingM4::remote_row_index(L, %d);\n",
            node->sanitized_name, node->sanitized_name, type_id);
  } else {
    fprintf(f, "    return load_function(L, %s_meta, ARRAY_SIZE(%s_meta));\n", node->sanitized_name, node->sanitized_name);
  }
  fprintf(f, "}\n\n");
}

// one local blob type on the M4: the in-process code of its fields, listed methods and
// operators, its codecs (no setter ranges) and its index; allocators and checkers come first
static void remote_emit_local_blob(FILE *f, int type_index, int *wire_size_emitted) {
  struct remote_type *rt = &remote_types[type_index];
  struct userdata *node = rt->node;
  for (struct userdata_field *field = node->fields; field != NULL; field = field->next) {
    emit_userdata_field(node, field);
  }
  for (struct method *method = node->methods; method != NULL; method = method->next) {
    if (remote_method_is_local(node, method)) {
      emit_userdata_method(node, method);
    }
  }
  if (node->operations) {
    emit_operators(node);
  }
  remote_emit_wire_size(f, type_index, wire_size_emitted);
  remote_emit_codec(f, rt);
  fprintf(f, "static void *create_%s(lua_State *L) {\n    return new_%s(L);\n}\n\n", node->sanitized_name, node->sanitized_name);
  fprintf(f, "static_assert(std::is_trivially_copyable<%s>::value, \"a local type is written back as bytes\");\n", node->name);
  fprintf(f, "static_assert(sizeof(%s) <= ScriptingM4::LOCAL_OBJECT_MAX, \"%s is larger than a local object\");\n",
          node->name, node->name);
  fprintf(f, "static_assert(wire_size_%s(nullptr) <= ScriptingM4::LOCAL_WIRE_MAX, \"%s's wire form is larger than a local wire buffer\");\n\n",
          node->sanitized_name, node->name);
  remote_emit_local_index(f, rt, type_index + 1);
}

// the M4's local types: uint32_t and uint64_t (the in-process code, with m4/m4_binding_deps.h for
// AP's headers), and the local blob types (AP_Math and AP_Common/Location, m4/local)
static void remote_emit_local(FILE *f) {
  FILE *saved = source;
  source = f;
  suppress_dependency_guards = TRUE;
  remote_codec_ranges = FALSE;
  fprintf(f, "// auto generated by gen-bindings -r, don't manually edit.  See README.md for details.\n\n");
  fprintf(f, "#pragma GCC optimize(\"Os\")\n");
  fprintf(f, "#include <type_traits>\n");
  fprintf(f, "#include \"m4_binding_deps.h\"\n");
  fprintf(f, "#include <generator/gen/generated_cpp_deps.h>\n\n");
  for (int i = 0; i < remote_type_count; i++) {
    if ((remote_types[i].kind == REMOTE_LOCAL) || (remote_types[i].kind == REMOTE_LOCAL_BLOB)) {
      emit_userdata_allocator(remote_types[i].node);
      emit_userdata_checker(remote_types[i].node);
    }
  }
  for (int i = 0; i < remote_type_count; i++) {
    struct userdata *node = remote_types[i].node;
    if (remote_types[i].kind != REMOTE_LOCAL) {
      continue;
    }
    for (struct method *method = node->methods; method != NULL; method = method->next) {
      emit_userdata_method(node, method);
    }
    if (node->operations) {
      emit_operators(node);
    }
    emit_index_node(node);
  }
  int wire_size_emitted[REMOTE_MAX_TYPES] = { 0 };
  int local_blob_count = 0;
  for (int i = 0; i < remote_type_count; i++) {
    if (remote_types[i].kind == REMOTE_LOCAL_BLOB) {
      remote_emit_local_blob(f, i, wire_size_emitted);
      local_blob_count++;
    }
  }
  fprintf(f, "static const struct userdata_meta remote_local_types[] = {\n");
  for (int i = 0; i < remote_type_count; i++) {
    const struct userdata *node = remote_types[i].node;
    if ((remote_types[i].kind != REMOTE_LOCAL) && (remote_types[i].kind != REMOTE_LOCAL_BLOB)) {
      continue;
    }
    if (node->operations) {
      fprintf(f, "    { \"%s\", %s_index, %s_operators },\n", remote_types[i].lua_name, node->sanitized_name, node->sanitized_name);
    } else {
      fprintf(f, "    { \"%s\", %s_index, nullptr },\n", remote_types[i].lua_name, node->sanitized_name);
    }
  }
  fprintf(f, "};\n\n");
  fprintf(f, "bool remote_local_type(const char *name, lua_CFunction &index, const luaL_Reg *&operators)\n{\n");
  fprintf(f, "    for (const struct userdata_meta &type : remote_local_types) {\n");
  fprintf(f, "        if (strcmp(type.name, name) == 0) {\n");
  fprintf(f, "            index = type.func;\n");
  fprintf(f, "            operators = type.operators;\n");
  fprintf(f, "            return true;\n");
  fprintf(f, "        }\n");
  fprintf(f, "    }\n");
  fprintf(f, "    return false;\n");
  fprintf(f, "}\n\n");
  if (local_blob_count > 0) {
    fprintf(f, "static const ScriptingM4::LocalCodec remote_local_codecs[] = {\n");
    for (int i = 0; i < remote_type_count; i++) {
      const struct userdata *node = remote_types[i].node;
      if (remote_types[i].kind == REMOTE_LOCAL_BLOB) {
        fprintf(f, "    { %d, sizeof(%s), uint16_t(wire_size_%s(nullptr)), create_%s, decode_%s, encode_%s },\n",
                i + 1, node->name, node->sanitized_name, node->sanitized_name, node->sanitized_name, node->sanitized_name);
      }
    }
    fprintf(f, "};\n\n");
  }
  fprintf(f, "const ScriptingM4::LocalCodec *ScriptingM4::local_codec(uint8_t type_id)\n{\n");
  if (local_blob_count > 0) {
    fprintf(f, "    for (const LocalCodec &codec : remote_local_codecs) {\n");
    fprintf(f, "        if (codec.type_id == type_id) {\n            return &codec;\n        }\n    }\n");
  } else {
    fprintf(f, "    (void)type_id;\n");
  }
  fprintf(f, "    return nullptr;\n}\n\n");
  fprintf(f, "lua_CFunction ScriptingM4::remote_local_constructor(uint8_t type_id)\n{\n    switch (type_id) {\n");
  for (int i = 0; i < remote_type_count; i++) {
    const struct userdata *node = remote_types[i].node;
    if ((remote_types[i].kind != REMOTE_LOCAL) && (remote_types[i].kind != REMOTE_LOCAL_BLOB)) {
      continue;
    }
    if (node->creation != NULL) {
      fprintf(f, "    case %d:\n        return %s;\n", i + 1, node->creation);
    } else if (should_emit_creation((struct userdata *)node)) {
      fprintf(f, "    case %d:\n        return lua_new_%s;\n", i + 1, node->sanitized_name);
    }
  }
  fprintf(f, "    default:\n        return nullptr;\n    }\n}\n");
  suppress_dependency_guards = FALSE;
  remote_codec_ranges = TRUE;
  source = saved;
}

static void remote_emit_blob_support(FILE *f, const struct remote_type *rt, int type_id, int with_constructor) {
  struct userdata *node = rt->node;
  start_dependency(f, node->dependency);
  fprintf(f, "%s * check_%s(lua_State *L, int arg) {\n", node->name, node->sanitized_name);
  fprintf(f, "    return static_cast<%s *>(remote_check_blob(L, arg, %d));\n}\n\n", node->name, type_id);
  fprintf(f, "%s * new_%s(lua_State *L) {\n", node->name, node->sanitized_name);
  fprintf(f, "    void *ud = remote_new_blob(L, %d, sizeof(%s));\n", type_id, node->name);
  fprintf(f, "    new (ud) %s();\n", node->name);
  fprintf(f, "    return static_cast<%s *>(ud);\n}\n\n", node->name);
  if (with_constructor && (node->creation == NULL) && should_emit_creation(node)) {
    fprintf(f, "static int lua_new_%s(lua_State *L) {\n", node->sanitized_name);
    fprintf(f, "    static bool warned = false;\n");
    fprintf(f, "    if (!warned && userdata_zero_arg_check(L)) {\n");
    fprintf(f, "        warned = true;\n");
    fprintf(f, "    }\n");
    fprintf(f, "    new_%s(L);\n", node->sanitized_name);
    fprintf(f, "    return 1;\n}\n\n");
  }
  end_dependency(f, node->dependency);
}

static void remote_emit_handle_support(FILE *f, const struct remote_type *rt, int type_id) {
  struct userdata *node = rt->node;
  start_dependency(f, node->dependency);
  fprintf(f, "%s ** check_%s(lua_State *L, int arg) {\n", node->name, node->sanitized_name);
  fprintf(f, "    return reinterpret_cast<%s **>(remote_check_handle(L, arg, %d));\n}\n\n", node->name, type_id);
  fprintf(f, "%s ** new_%s(lua_State *L) {\n", node->name, node->sanitized_name);
  fprintf(f, "    return reinterpret_cast<%s **>(remote_new_handle(L, %d));\n}\n\n", node->name, type_id);
  end_dependency(f, node->dependency);
}

// an owned type: the object lives in the M7's object pool; check_ returns it, new_ makes it there
static void remote_emit_owned_support(FILE *f, const struct remote_type *rt, int type_id) {
  struct userdata *node = rt->node;
  const char *release = remote_owned_release_of(node->name);
  start_dependency(f, node->dependency);
  fprintf(f, "%s * check_%s(lua_State *L, int arg) {\n", node->name, node->sanitized_name);
  fprintf(f, "    return static_cast<%s *>(remote_check_owned(L, arg, %d));\n}\n\n", node->name, type_id);
  fprintf(f, "%s * new_%s(lua_State *L) {\n", node->name, node->sanitized_name);
  fprintf(f, "    static_assert(sizeof(%s) <= ScriptingM4::ObjectPool::SLOT_SIZE && alignof(%s) <= ScriptingM4::ObjectPool::SLOT_ALIGN,\n",
          node->name, node->name);
  fprintf(f, "                  \"%s fits an object pool slot\");\n", node->name);
  if (release != NULL) {
    fprintf(f, "    void *ud = remote_new_owned(L, %d, %s);\n", type_id, release);
  } else {
    fprintf(f, "    void *ud = remote_new_owned(L, %d, ScriptingM4::destroy_object<%s>);\n", type_id, node->name);
  }
  fprintf(f, "    new (ud) %s();\n", node->name);
  fprintf(f, "    return static_cast<%s *>(ud);\n}\n\n", node->name);
  end_dependency(f, node->dependency);
}

// the #if of a table entry: the owning type's and the method's own depends
static const char *remote_dependency(const struct remote_method *method) {
  const char *type_dependency = remote_types[method->type - 1].node->dependency;
  if ((type_dependency != NULL) && (method->dependency != NULL)) {
    return remote_format("(%s) && (%s)", type_dependency, method->dependency);
  }
  return (type_dependency != NULL) ? type_dependency : method->dependency;
}

static void remote_emit_entry(FILE *f, const char *dependency, const char *value, const char *otherwise) {
  if (dependency == NULL) {
    fprintf(f, "    %s,\n", value);
  } else {
    fprintf(f, "#if %s\n    %s,\n#else\n    %s,\n#endif\n", dependency, value, otherwise);
  }
}

// the M7 dispatch: the in-process binding functions against AP_ScriptingM4_Remote.h
static void remote_emit_dispatch(FILE *f) {
  FILE *saved = source;
  source = f;
  fprintf(f, "// auto generated by gen-bindings -r, don't manually edit.  See README.md for details.\n");
  fprintf(f, "#include <AP_Scripting/AP_Scripting_config.h>\n\n#if AP_SCRIPTING_M4_ENABLED\n\n");
  fprintf(f, "#pragma GCC optimize(\"Os\")\n#include <new>\n#include <type_traits>\n");
  fprintf(f, "#include <AP_Vehicle/AP_Vehicle_Type.h> // needed for APM_BUILD_TYPE #if\n");
  fprintf(f, "#include <AP_Scheduler/AP_Scheduler.h>\n");
  fprintf(f, "#include <AP_Scripting/AP_Scripting_BindingStats.h>\n");
  fprintf(f, "#include <AP_Scripting/AP_ScriptingM4_Bindings.h>\n");
  fprintf(f, "#include <AP_Scripting/AP_ScriptingM4_Quota.h>\n");
  fprintf(f, "#include <AP_Scripting/AP_ScriptingM4_Remote.h>\n");
  // twice, as in-process (lua_generated_bindings.h, then .cpp): a header's #if (OSD_ENABLED)
  // can depend on a config header that only a later header of the list includes
  emit_headers(f);
  emit_headers(f);
  fprintf(f, "\n");
  fprintf(f, "extern const AP_HAL::HAL& hal;\n\n");
  fprintf(f, "namespace ScriptingM4Remote {\n\n");
  fprintf(f, "using ScriptingM4::ReplyStatus;\nusing ScriptingM4::RpcValue;\nusing ScriptingM4::RpcWriter;\n\n");
  // the manual functions in remote_ready_manual[] are declared in AP_ScriptingM4_Remote.h

  // declarations first: the M7 build warns about definitions without one (-Wmissing-declarations)
  for (int i = 0; i < remote_type_count; i++) {
    const struct remote_type *rt = &remote_types[i];
    const struct userdata *node = rt->node;
    if ((rt->kind == REMOTE_SINGLETON) && !(node->flags & UD_FLAG_LITERAL) && (node->methods != NULL)) {
      start_dependency(f, node->dependency);
      fprintf(f, "%s * check_%s(lua_State *L);\n", node->name, node->sanitized_name);
      end_dependency(f, node->dependency);
    } else if (remote_kind_is_wire(rt->kind) || (rt->kind == REMOTE_HANDLE) || (rt->kind == REMOTE_OWNED)) {
      const char *pointer = (rt->kind == REMOTE_HANDLE) ? "**" : "*";
      start_dependency(f, node->dependency);
      fprintf(f, "%s %s check_%s(lua_State *L, int arg);\n", node->name, pointer, node->sanitized_name);
      fprintf(f, "%s %s new_%s(lua_State *L);\n", node->name, pointer, node->sanitized_name);
      end_dependency(f, node->dependency);
    }
  }
  fprintf(f, "\n");

  // userdata storage: check_<type>() and new_<type>()
  for (int i = 0; i < remote_type_count; i++) {
    const struct remote_type *rt = &remote_types[i];
    switch (rt->kind) {
      case REMOTE_SINGLETON:
        emit_singleton_checker(rt->node);
        break;
      case REMOTE_BLOB:
        remote_emit_blob_support(f, rt, i + 1, TRUE);
        break;
      case REMOTE_LOCAL_BLOB:
        // other bindings take and return it; its constructor is the M4's
        remote_emit_blob_support(f, rt, i + 1, FALSE);
        break;
      case REMOTE_HANDLE:
        remote_emit_handle_support(f, rt, i + 1);
        break;
      case REMOTE_OWNED:
        remote_emit_owned_support(f, rt, i + 1);
        break;
      case REMOTE_LOCAL:
      case REMOTE_GLOBAL:
        break;
    }
  }

  // wire codecs; nested types refer to each other
  for (int i = 0; i < remote_type_count; i++) {
    const struct remote_type *rt = &remote_types[i];
    if (remote_kind_is_wire(rt->kind)) {
      start_dependency(f, rt->node->dependency);
      fprintf(f, "static bool read_%s(WireReader &r, %s *ud);\n", rt->node->sanitized_name, rt->node->name);
      fprintf(f, "static void write_%s(WireWriter &w, const %s *ud);\n", rt->node->sanitized_name, rt->node->name);
      end_dependency(f, rt->node->dependency);
    }
  }
  fprintf(f, "\n");
  int wire_size_emitted[REMOTE_MAX_TYPES] = { 0 };
  for (int i = 0; i < remote_type_count; i++) {
    if (remote_kind_is_wire(remote_types[i].kind)) {
      remote_emit_wire_size(f, i, wire_size_emitted);
    }
  }
  fprintf(f, "\n");
  for (int i = 0; i < remote_type_count; i++) {
    if (remote_kind_is_wire(remote_types[i].kind)) {
      remote_emit_codec(f, &remote_types[i]);
    }
  }

  // the binding functions, as in-process
  for (int i = 0; i < remote_type_count; i++) {
    const struct remote_type *rt = &remote_types[i];
    struct userdata *node = rt->node;
    if ((rt->kind != REMOTE_SINGLETON) && !remote_kind_is_wire(rt->kind) && (rt->kind != REMOTE_HANDLE) &&
        (rt->kind != REMOTE_OWNED)) {
      continue;
    }
    // a local type's fields and operators are the M4's own
    if (rt->kind != REMOTE_LOCAL_BLOB) {
      for (struct userdata_field *field = node->fields; field != NULL; field = field->next) {
        if (!remote_type_supported(&field->type)) {
          continue;
        }
        start_dependency(f, node->dependency);
        if (rt->kind == REMOTE_SINGLETON) {
          emit_singleton_field(node, field);
        } else if ((rt->kind == REMOTE_BLOB) || (rt->kind == REMOTE_OWNED)) {
          emit_userdata_field(node, field);
        } else {
          emit_ap_object_field(node, field);
        }
        end_dependency(f, node->dependency);
      }
    }
    for (struct method *method = node->methods; method != NULL; method = method->next) {
      if (remote_method_supported(method) && !remote_method_is_local(node, method) &&
          (remote_replacement_of(node, method) == NULL)) {
        emit_userdata_method(node, method);
      }
    }
    if ((rt->kind == REMOTE_BLOB) && node->operations) {
      emit_operators(node);
    }
  }

  // tables by method id and type id; what the board left out is nullptr, false or 0
  int max_quota = 0;
  fprintf(f, "static const lua_CFunction remote_functions[ScriptingM4::BINDING_METHOD_COUNT] = {\n");
  for (int i = 0; i < remote_method_count; i++) {
    const struct remote_method *method = &remote_methods[i];
    if (method->used) {
      remote_emit_entry(f, remote_dependency(method), method->function, "nullptr");
      if ((method->flags >> 4) > max_quota) {
        max_quota = method->flags >> 4;
      }
    } else {
      fprintf(f, "    nullptr,\n");
    }
  }
  fprintf(f, "};\n\nstatic const uint8_t remote_method_flags[ScriptingM4::BINDING_METHOD_COUNT] = {\n");
  for (int i = 0; i < remote_method_count; i++) {
    fprintf(f, "    0x%02x,\n", remote_methods[i].used ? remote_methods[i].flags : 0);
  }
  fprintf(f, "};\n// the high nibble of a flag indexes the quota table\n");
  fprintf(f, "static_assert(%d < ScriptingM4::QUOTA_CLASSES, \"a method's quota class is past the quota table\");\n", max_quota);
  fprintf(f, "\nstatic const BlobCodec remote_codecs[ScriptingM4::BINDING_TYPE_COUNT] = {\n");
  for (int i = 0; i < remote_type_count; i++) {
    const struct remote_type *rt = &remote_types[i];
    char *none = remote_format("{ \"%s\", 0, nullptr, nullptr }", rt->lua_name);
    if (remote_kind_is_wire(rt->kind)) {
      remote_emit_entry(f, rt->node->dependency,
                        remote_format("{ \"%s\", sizeof(%s), decode_%s, encode_%s }", rt->lua_name, rt->node->name,
                                      rt->node->sanitized_name, rt->node->sanitized_name),
                        none);
    } else {
      fprintf(f, "    %s,\n", none);
    }
  }
  fprintf(f, "};\n\nstatic const bool remote_type_available[ScriptingM4::BINDING_TYPE_COUNT] = {\n");
  for (int i = 0; i < remote_type_count; i++) {
    remote_emit_entry(f, remote_types[i].node->dependency, "true", "false");
  }
  fprintf(f, "};\n\nstatic const int32_t remote_enum_values[ScriptingM4::BINDING_ENUM_COUNT] = {\n");
  for (int i = 0; i < remote_enum_count; i++) {
    const struct userdata *node = remote_types[remote_enums[i].type - 1].node;
    remote_emit_entry(f, node->dependency, remote_format("int32_t(%s::%s)", node->name, remote_enums[i].name), "0");
  }
  fprintf(f, "};\n\n");
  fputs("bool method_available(uint16_t method)\n"
        "{\n"
        "    const uint16_t index = uint16_t(method - ScriptingM4::FIRST_BINDING_METHOD);\n"
        "    return method >= ScriptingM4::FIRST_BINDING_METHOD && index < ScriptingM4::BINDING_METHOD_COUNT &&\n"
        "           remote_functions[index] != nullptr;\n"
        "}\n\n"
        "uint8_t method_flags(uint16_t method)\n"
        "{\n"
        "    return method_available(method) ? remote_method_flags[method - ScriptingM4::FIRST_BINDING_METHOD] : 0;\n"
        "}\n\n"
        "// scripting host thread only: run() keeps the call in a static, and the handle\n"
        "// table's reset(session) must never run while a call is in progress\n"
        "ReplyStatus dispatch(uint16_t method, const RpcValue *args, uint8_t argc, RemoteCall &call,\n"
        "                     RpcWriter &results, uint8_t &error_arg)\n"
        "{\n"
        "    if (!method_available(method)) {\n"
        "        return ReplyStatus::UnknownMethod;\n"
        "    }\n"
        "    const uint16_t index = uint16_t(method - ScriptingM4::FIRST_BINDING_METHOD);\n"
        "    call.codecs = remote_codecs;\n"
        "    call.codec_count = ScriptingM4::BINDING_TYPE_COUNT;\n"
        "    call.uint32_type = ScriptingM4::BINDING_TYPE_UINT32;\n"
        "    call.uint64_type = ScriptingM4::BINDING_TYPE_UINT64;\n"
        "    return run(call, args, argc, remote_functions[index], remote_method_flags[index], results, error_arg);\n"
        "}\n\n"
        "void binding_info(uint8_t *info)\n"
        "{\n"
        "    memset(info, 0, ScriptingM4::BINDING_INFO_SIZE);\n"
        "    for (uint16_t i = 0; i < ScriptingM4::BINDING_METHOD_COUNT; i++) {\n"
        "        if (remote_functions[i] != nullptr) {\n"
        "            ScriptingM4::info_set_method(info, uint16_t(ScriptingM4::FIRST_BINDING_METHOD + i));\n"
        "        }\n"
        "    }\n"
        "    for (uint8_t i = 0; i < ScriptingM4::BINDING_TYPE_COUNT; i++) {\n"
        "        if (remote_type_available[i]) {\n"
        "            ScriptingM4::info_set_type(info, uint8_t(i + 1));\n"
        "        }\n"
        "    }\n"
        "    for (uint16_t i = 0; i < ScriptingM4::BINDING_ENUM_COUNT; i++) {\n"
        "        ScriptingM4::info_set_enum(info, i, remote_enum_values[i]);\n"
        "    }\n"
        "}\n\n", f);
  fprintf(f, "}  // namespace ScriptingM4Remote\n\n#endif  // AP_SCRIPTING_M4_ENABLED\n");
  source = saved;
}

static void remote_write_files(const char *prefix) {
  FILE *f = remote_open(prefix, "contract.h");
  remote_emit_contract(f);
  fclose(f);
  f = remote_open(prefix, "tables.cpp");
  remote_emit_tables(f);
  fclose(f);
  f = remote_open(prefix, "local.cpp");
  remote_emit_local(f);
  fclose(f);
  f = remote_open(prefix, "dispatch.cpp");
  remote_emit_dispatch(f);
  fclose(f);
}

char * output_path = NULL;
char * docs_path = NULL;
char * remote_path = NULL;

int main(int argc, char **argv) {
  state.line_num = -1;

  int c;
  while ((c = getopt(argc, argv, "i:o:d:r:")) != -1) {
    switch (c) {
      case 'i':
        if (description != NULL) {
          error(ERROR_GENERAL, "Already loaded a description file");
        }
        trace(TRACE_GENERAL, "Loading a description file: %s", optarg);
        description = fopen(optarg, "r");
        if (description == NULL) {
          error(ERROR_GENERAL, "Unable to load the description file: %s", optarg);
        }
        break;
      case 'o':
        if (output_path != NULL) {
          error(ERROR_GENERAL, "An output path was already selected.");
        }
        output_path = optarg;
        trace(TRACE_GENERAL, "Loading an output path of %s", output_path);
        break;
      case 'd':
        if (docs_path != NULL) {
          error(ERROR_GENERAL, "An docs path was already selected.");
        }
        docs_path = optarg;
        trace(TRACE_GENERAL, "Loading an docs path of %s", docs_path);
        break;
      case 'r':
        if (remote_path != NULL) {
          error(ERROR_GENERAL, "A remote output path was already selected.");
        }
        remote_path = optarg;
        break;
    }
  }

  if (output_path == NULL) {
    error(ERROR_GENERAL, "An output path must be provided for the generated bindings");
  }

  state.line_num = 0;

  while (start_line()) {

    // identify line type
    if (state.token != NULL) {
      // we have input
      if (strcmp(state.token, keyword_comment) == 0) {
        while (next_token()) {}
        // nothing to do here, jump to the next line
      } else if (strcmp(state.token, keyword_include) == 0) {
        handle_header();
      } else if (strcmp (state.token, keyword_userdata) == 0){
        handle_userdata();
      } else if (strcmp (state.token, keyword_singleton) == 0){
        handle_singleton();
      } else if (strcmp (state.token, keyword_ap_object) == 0){
        handle_ap_object();
      } else if (strcmp (state.token, keyword_global) == 0){
        handle_global();
      } else {
        error(ERROR_UNKNOWN_KEYWORD, "Expected a keyword, got: %s", state.token);
      }

      if (next_token()) {
        error(ERROR_UNKNOWN_KEYWORD, "Extra token provided: %s", state.token);
      }
    }
  }

  state.line_num = -1;

  char *file_name = (char *)allocate(strlen(output_path) + 5);
  sprintf(file_name, "%s.cpp", output_path);
  source = fopen(file_name, "w");
  if (source == NULL) {
    error(ERROR_GENERAL, "Unable to open the output source file: %s", file_name);
  }

  fprintf(source, "// auto generated bindings, don't manually edit.  See README.md for details.\n");

  fprintf(source, "#include <AP_Scripting/AP_Scripting_config.h>\n\n");

  fprintf(source, "#if AP_SCRIPTING_ENABLED\n\n");

  trace(TRACE_GENERAL, "Sanity checking parsed input");

  sanity_check_userdata();

  fprintf(source, "#pragma GCC optimize(\"Os\")\n");
  fprintf(source, "#include \"lua_generated_bindings.h\"\n");
  fprintf(source, "#include <AP_Scripting/lua_boxed_numerics.h>\n");
  fprintf(source, "#include <AP_Scripting/lua_bindings.h>\n");
  fprintf(source, "#include <AP_Scripting/AP_Scripting_BindingStats.h>\n");

  // for set_and_print_new_error_message deprecate warning
  fprintf(source, "#include <AP_Scripting/lua_scripts.h>\n");
  // for the various utility methods
  fprintf(source, "#include <AP_Scripting/generator/gen/generated_cpp_deps.h>\n\n");

  // the generated source uses the Scheduler singleton:
  fprintf(source, "#include <AP_Scheduler/AP_Scheduler.h>\n");
  fprintf(source, "extern const AP_HAL::HAL& hal;\n");

  trace(TRACE_GENERAL, "Starting emission");

  emit_headers(source);

  fprintf(source, "\n\n");

  emit_userdata_allocators();

  emit_userdata_checkers();

  emit_ap_object_allocators();

  emit_ap_object_checkers();

  emit_singleton_checkers();

  emit_singleton_fields();
  emit_methods(parsed_singletons);
  emit_index(parsed_singletons);

  emit_userdata_fields();
  emit_methods(parsed_userdata);
  emit_index(parsed_userdata);

  emit_ap_object_fields();
  emit_methods(parsed_ap_objects);
  emit_index(parsed_ap_objects);

  emit_userdata_new_funcs();
  emit_loaders();

  fprintf(source, "#if AP_SCRIPTING_BINDING_STATS_ENABLED\n");
  fprintf(source, "extern const uint16_t binding_stats_count;\n");
  fprintf(source, "extern const char *const binding_stats_names[];\n");
  fprintf(source, "const uint16_t binding_stats_count = %d;\n", next_stats_id);
  fprintf(source, "const char *const binding_stats_names[] = {\n");
  for (int i = 0; i < next_stats_id; i++) {
    fprintf(source, "    \"%s\",\n", stats_names[i]);
  }
  fprintf(source, "};\n#endif\n\n");

  // after everything is defined
  fprintf(source, "#include <AP_Scripting/generator/gen/generated_cpp_loader.h>\n\n");

  fprintf(source, "#endif  // AP_SCRIPTING_ENABLED\n");

  fclose(source);
  source = NULL;

  sprintf(file_name, "%s.h", output_path);
  header = fopen(file_name, "w");
  if (header == NULL) {
    error(ERROR_GENERAL, "Unable to open the output header file: %s", file_name);
  }
  free(file_name);
  fprintf(header, "#pragma once\n");
  fprintf(header, "// auto generated bindings, don't manually edit.  See README.md for details.\n");
  fprintf(header, "#include <AP_Scripting/AP_Scripting_config.h>\n\n");
  fprintf(header, "#if AP_SCRIPTING_ENABLED\n\n");
  fprintf(header, "#include <AP_Vehicle/AP_Vehicle_Type.h> // needed for APM_BUILD_TYPE #if\n");
  emit_headers(header);
  fprintf(header, "\n#include <AP_Scripting/lua/src/lua.hpp>\n");
  fprintf(header, "#include <AP_Scripting/generator/gen/generated_h_deps.h>\n");
  fprintf(header, "#include <new>\n\n");

  emit_userdata_declarations();
  emit_ap_object_declarations();

  struct userdata * node = parsed_singletons;
  while (node) {
    if (!(node->flags & UD_FLAG_LITERAL) && (node->methods != NULL)) {
      start_dependency(header, node->dependency);
      fprintf(header, "%s * check_%s(lua_State *L);\n", node->name, node->sanitized_name);
      end_dependency(header, node->dependency);
    }
    node = node->next;
  }

  fprintf(header, "#endif  // AP_SCRIPTING_ENABLED\n");

  fclose(header);
  header = NULL;

  if (remote_path != NULL) {
    remote_collect();
    remote_write_files(remote_path);
  }

  if (docs_path == NULL) {
    // no docs to generate, all done
    return 0;
  }

  docs = fopen(docs_path, "w");
  if (docs == NULL) {
    error(ERROR_GENERAL, "Unable to open the output docs file: %s", docs_path);
  }

  fprintf(docs, "-- ArduPilot lua scripting documentation in EmmyLua Annotations\n");
  fprintf(docs, "-- This file should be auto generated and then manual edited\n");
  fprintf(docs, "-- generate with scripting-docs, eg  ./waf scripting-docs\n");
  fprintf(docs, "-- see: https://github.com/sumneko/lua-language-server/wiki/EmmyLua-Annotations\n");
  fprintf(docs, "-- luacheck: ignore 121 (Setting a read-only global variable)\n");
  fprintf(docs, "-- luacheck: ignore 122 (Setting a read-only field of a global variable)\n");
  fprintf(docs, "-- luacheck: ignore 212 (Unused argument)\n");
  fprintf(docs, "-- luacheck: ignore 241 (Local variable is mutated but never accessed)\n\n");

  emit_docs(parsed_userdata, TRUE, TRUE);

  emit_docs(parsed_ap_objects, TRUE, FALSE);

  emit_docs(parsed_singletons, FALSE, FALSE);

  // global aliases
  if (parsed_globals != NULL) {
    struct method_alias *alias = parsed_globals->method_aliases;
    while(alias) {
      if (alias->type == ALIAS_TYPE_MANUAL) {
          // Cant do a great job, don't know types or return
          fprintf(docs, "-- desc\n");

          for (int i = 0; i < alias->num_args; ++i) {
            fprintf(docs, "---@param param%i UNKNOWN\n", i+1);
          }

          for (int i = 0; i < alias->num_ret; ++i) {
            fprintf(docs, "---@return UNKNOWN\n");
          }

          fprintf(docs, "function %s(", alias->alias);
          for (int i = 0; i < alias->num_args; ++i) {
            fprintf(docs, "param%i", i+1);
            if (i < alias->num_args-1) {
              fprintf(docs, ", ");
            }
          }
          fprintf(docs, ") end\n\n");
      }
      alias = alias->next;
    }
  }

  fclose(docs);

  return 0;
}
