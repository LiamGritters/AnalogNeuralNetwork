/*
 * CreateXORData.cpp
 *
 *  Created on: 2019-01-31
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "hdf5/serial/hdf5.h"
#include "hdf5/serial/H5Cpp.h"

#include <string>
#include <iostream>

/****************************************
 * NAMESPACE
 ****************************************/

using namespace H5;

/****************************************
 * CONSTANTS
 ****************************************/

const H5std_string  FILE_NAME( "xor_data.hdf5" );
const H5std_string  DATASET_NAME( "data" );
const H5std_string  LABELSET_NAME( "label" );
const int   NX = 8;                    // dataset dimensions
const int   NY = 2;
const int   RANK = 2;

/****************************************
 * Main
 ****************************************/

int main (void)
{
   /*
    * Data initialization.
    */
   float data[NX][NY] = { {0.0,0.0}, {0.0,1.0}, {1.0,0.0}, {1.0,1.0}, {1.0,1.0}, {0.0,1.0}, {0.0,1.0}, {1.0,1.0} };          // buffer for data to write
   float label[NX] = {0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0};


   // Try block to catch exceptions raised by any of the calls inside it
   try
   {
   // Turn off the auto-printing when failure occurs so that we can
   // handle the errors appropriately
   Exception::dontPrint();

   // Open an existing file and dataset.
   H5File file(FILE_NAME, H5F_ACC_TRUNC);

       // Create the data space for the first dataset.  Note the use of
       // pointer for the instance 'dataspace'.  It can be deleted and
       // used again later for another data space.  An HDF5 identifier is
       // closed by the destructor or the method 'close()'.
   hsize_t dims[RANK];               // dataset dimensions
   dims[0] = NX;
   dims[1] = NY;
   DataSpace *dataspace = new DataSpace (RANK, dims);

   // Create the dataset in group "MyGroup".  Same note as for the
   // dataspace above.
   DataSet *dataset = new DataSet (file.createDataSet(DATASET_NAME,
                                    PredType::NATIVE_FLOAT, *dataspace));

   // Write the data to the dataset using default memory space, file
   // space, and transfer properties.
   dataset->write(data, PredType::NATIVE_FLOAT);

   // Close the current dataset and data space.
   delete dataset;
   delete dataspace;

   // Create the data space for the second dataset.
   dims[0] = NX;
   dims[1] = 1;
   dataspace = new DataSpace (RANK, dims);

//   // Create group "Group_A" in group "MyGroup".
//   Group group(file.openGroup("/MyGroup/Group_A"));

   // Create the second dataset in group "Group_A".
   dataset = new DataSet (file.createDataSet(LABELSET_NAME,
                           PredType::NATIVE_FLOAT, *dataspace));

   // Write the data to the dataset using default memory space, file
   // space, and transfer properties.
   dataset->write(label, PredType::NATIVE_FLOAT);

   // Close all objects.
   delete dataspace;
   delete dataset;
   file.close();

   } // end of try block

   // catch failure caused by the H5File operations
   catch(FileIException error)
   {
    error.printErrorStack();
    return -1;
   }
   // catch failure caused by the DataSet operations
   catch(DataSetIException error)
   {
    error.printErrorStack();
    return -1;
   }

   // catch failure caused by the DataSpace operations
   catch(DataSpaceIException error)
   {
   error.printErrorStack();
   return -1;
   }

   // catch failure caused by the Group operations
   catch(GroupIException error)
   {
   error.printErrorStack();
   return -1;
   }

    return 0;
}





