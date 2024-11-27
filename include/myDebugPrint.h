
#define MYDEBUGON


#ifdef MYDEBUGON
  #define MYDEBUGPRINT(x)     		MYSERIALX.print (x)
  #define MYDEBUGPRINTDEC(x)  		MYSERIALX.print (x, DEC)
  #define MYDEBUGPRINTSTRING(x)  	MYSERIALX.print ("%s", x)
  #define MYDEBUGPRINTLN(x)  			MYSERIALX.println (x)
  #define MYDEBUGWRITE(x)     		MYSERIALX.write (x)
#else
  #define MYDEBUGPRINT(x)
  #define MYDEBUGPRINTDEC(x)
  #define MYDEBUGPRINTSTRING(x)
  #define MYDEBUGPRINTLN(x) 
  #define MYDEBUGWRITE(x)
#endif
