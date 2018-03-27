/*
 * sample_cvFileStorage.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: aras
 */

/*
XML

<?xml version="1.0">
<opencv_storage>
<A type_id="opencv-matrix">
  <rows>3</rows>
  <cols>3</cols>
  <dt>f</dt>
  <data>1. 0. 0. 0. 1. 0. 0. 0. 1.</data>
</A>
</opencv_storage>
*/



/*
YML
YAML:

%YAML:1.0
A: !!opencv-matrix
  rows: 3
  cols: 3
  dt: f
  data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1.]
  */


union
{
  double f; /* scalar floating-point number */
  int i;    /* scalar integer number */
  CvString str; /* text string */
  CvSeq* seq; /* sequence (ordered collection of file nodes) */
  struct CvMap* map; /* map (collection of named file nodes) */
} data;



//CvAttrList
//struct CvAttrList
//List of attributes.

typedef struct CvAttrList
{
  const char** attr; /* NULL-terminated array of (attribute_name,attribute_value) pairs */
  struct CvAttrList* next; /* pointer to next chunk of the attributes list */
}
CvAttrList;



/* initializes CvAttrList structure */
inline CvAttrList cvAttrList(const char** attr = NULL, CvAttrList* next = NULL);

/* returns attribute value or 0 (NULL) if there is no such attribute */
const char* cvAttrValue(const CvAttrList* attr, const char* attr_name);


////////////////////////////////////////////////////////////////////

/*
CvTypeInfo¶
struct CvTypeInfo
Type information.
*/

//typedef int (CV_CDECL *CvIsInstanceFunc)( const void* structPtr );
//typedef void (CV_CDECL *CvReleaseFunc)( void** structDblPtr );
//typedef void* (CV_CDECL *CvReadFunc)( CvFileStorage* storage, CvFileNode* node );
//typedef void (CV_CDECL *CvWriteFunc)( CvFileStorage* storage,
//                                      const char* name,
//                                      const void* structPtr,
//                                      CvAttrList attributes );
//typedef void* (CV_CDECL *CvCloneFunc)( const void* structPtr );

typedef struct CvTypeInfo
{
  int flags; /* not used */
  int header_size; /* sizeof(CvTypeInfo) */
  struct CvTypeInfo* prev; /* previous registered type in the list */
  struct CvTypeInfo* next; /* next registered type in the list */
  const char* type_name; /* type name, written to file storage */

  /* methods */
  CvIsInstanceFunc is_instance; /* checks if the passed object belongs to the type */
  CvReleaseFunc release; /* releases object (memory etc.) */
  CvReadFunc read; /* reads object from file storage */
  CvWriteFunc write; /* writes object to file storage */
  CvCloneFunc clone; /* creates a copy of the object */
}
CvTypeInfo;

/////////////////////////////////////////////////////////////////////////////////////

/*
points:
  - { x: 10, y: 10 }
  - { x: 20, y: 20 }
  - { x: 30, y: 30 }
  # ...
  */
//#include "cxcore.h"
#include <opencv/cxcore.h>
#include <opencv/cv.h>

#include "projects/stereo_traversability_experiments/def/tMLR.h"
//int main(int argc, char** argv)
int file_storage_read_c_style(int argc, const char** argv)
{
  CvFileStorage* fs = cvOpenFileStorage("points.yml", 0, CV_STORAGE_READ);
  CvStringHashNode* x_key = cvGetHashedNode(fs, "x", -1, 1);
  CvStringHashNode* y_key = cvGetHashedNode(fs, "y", -1, 1);
  CvFileNode* points = cvGetFileNodeByName(fs, 0, "points");

  if (CV_NODE_IS_SEQ(points->tag))
  {
    CvSeq* seq = points->data.seq;
    int i, total = seq->total;
    CvSeqReader reader;
    cvStartReadSeq(seq, &reader, 0);
    for (i = 0; i < total; i++)
    {
      CvFileNode* pt = (CvFileNode*)reader.ptr;
#if 1 /* faster variant */
      CvFileNode* xnode = cvGetFileNode(fs, pt, x_key, 0);
      CvFileNode* ynode = cvGetFileNode(fs, pt, y_key, 0);
      assert(xnode && CV_NODE_IS_INT(xnode->tag) &&
             ynode && CV_NODE_IS_INT(ynode->tag));
      int x = xnode->data.i; // or x = cvReadInt( xnode, 0 );
      int y = ynode->data.i; // or y = cvReadInt( ynode, 0 );
#elif 1 /* slower variant; does not use x_key & y_key */
      CvFileNode* xnode = cvGetFileNodeByName(fs, pt, "x");
      CvFileNode* ynode = cvGetFileNodeByName(fs, pt, "y");
      assert(xnode && CV_NODE_IS_INT(xnode->tag) &&
             ynode && CV_NODE_IS_INT(ynode->tag));
      int x = xnode->data.i; // or x = cvReadInt( xnode, 0 );
      int y = ynode->data.i; // or y = cvReadInt( ynode, 0 );
#else /* the slowest yet the easiest to use variant */
      int x = cvReadIntByName(fs, pt, "x", 0 /* default value */);
      int y = cvReadIntByName(fs, pt, "y", 0 /* default value */);
#endif
      CV_NEXT_SEQ_ELEM(seq->elem_size, reader);
      std::printf("
                }
                }
                  cvReleaseFileStorage( &fs );
                  return 0;
                }







                  /////////////////////////////////////////////////
                  /*
                  StartNextStream
                  Starts the next stream.
                  */

                  /*
                  C: void cvStartNextStream(CvFileStorage* fs)
                  Parameters:
                  fs – File storage
                  The function finishes the currently written stream and starts the next stream. In the case of XML the file with multiple streams looks like this:
                  */

                  //<opencv_storage>
                  //<!-- stream #1 data -->
                  //</opencv_storage>
                  //<opencv_storage>
                  //<!-- stream #2 data -->
                  //</opencv_storage>
                  //...

                  /*The YAML file will look like this:*/

                  //%YAML:1.0
                  //# stream #1 data
                  //...
                  //---
                  //# stream #2 data

                  /////////////////////////////////////////////////////


                  //             int main( int argc, char** argv )
                  int file_storage_write_c_style(int argc, const char** argv)
                {
                  CvMat* mat = cvCreateMat( 3, 3, CV_32F );
                  CvFileStorage* fs = cvOpenFileStorage( "example.yml", 0, CV_STORAGE_WRITE );

                  cvSetIdentity( mat );
                  cvWrite( fs, "A", mat, cvAttrList(0,0) );

                  cvReleaseFileStorage( &fs );
                  cvReleaseMat( &mat );
                  return 0;
                }


                  /////////////////////////////////////////////////////////////

                  void write_termcriteria( CvFileStorage* fs, const char* struct_name,
                  CvTermCriteria* termcrit )
                {
                  cvStartWriteStruct( fs, struct_name, CV_NODE_MAP, NULL, cvAttrList(0,0));
                  cvWriteComment( fs, "termination criteria", 1 ); // just a description
                  if( termcrit->type & CV_TERMCRIT_ITER )
                  cvWriteInteger( fs, "max_iterations", termcrit->max_iter );
                  if( termcrit->type & CV_TERMCRIT_EPS )
                  cvWriteReal( fs, "accuracy", termcrit->epsilon );
                  cvEndWriteStruct( fs );
                }
