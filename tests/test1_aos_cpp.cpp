/*
  Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
  All rights reserved.

  Redistribution and use in source and binary forms,
  with or without modification, are permitted provided that the
  following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the Sony Computer Entertainment Inc nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#define _VECTORMATH_AOS_TEST
#define _VECTORMATH_DEBUG

#include "vectormath_aos.h"


int iteration = 0;

FILE* goldReference=0;
float epsilon=1e-4f;
int totalFailedTests = 0;
int totalPassedTests = 0;
#include "test.h"


using namespace Vectormath;
using namespace Vectormath::Aos;

void
Vector3_methods_test()
{
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3, e_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4, e_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3, e_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat, e_Quat;
    Vector4 tmpV4;
    Vector3 aos_Vector3_0, aos_Vector3_1, aos_Vector3_2, aos_Vector3_3;
    Vector4 aos_Vector4_0, aos_Vector4_1, aos_Vector4_2;
    float rndflt1, rndflt2, rndflt3, rndflt4, pad;
    VM_ALIGNED16(float xyz4[12]);

#ifndef _VECTORMATH_SCALAR_TEST
    vec_float4 quad;
#endif
    xyz4[0] = randfloat();
    xyz4[1] = randfloat();
    xyz4[2] = randfloat();
    xyz4[3] = randfloat();
    xyz4[4] = randfloat();
    xyz4[5] = randfloat();
    xyz4[6] = randfloat();
    xyz4[7] = randfloat();
    xyz4[8] = randfloat();
    xyz4[9] = randfloat();
    xyz4[10] = randfloat();
    xyz4[11] = randfloat();
    // set a pad value to detect invalid use of padding.
    // this will be nan for scalar/ppu implementations, max. float for spu
    union { float f; unsigned int u; } tmp;
    tmp.u = 0x7fffffff;
    pad = tmp.f;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Vector3 = Vector3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( a_Vector3, pad );
    a_Vector3 = tmpV4.getXYZ( );
    tmpV4 = Vector4( b_Vector3, pad );
    b_Vector3 = tmpV4.getXYZ( );
    
	testvm( a_Vector3, "set Vector3 with floats" );

    testvm( b_Vector3, "set Vector3 with floats" );
    c_Vector3 = Vector3( 0.0f );
    d_Vector3 = Vector3( 0.0f );
    e_Vector3 = Vector3( 0.0f );
    testvm( c_Vector3, "set Vector3 elements to zero" );
    testvm( d_Vector3, "set Vector3 elements to zero" );
    testvm( e_Vector3, "set Vector3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Vector4 = Vector4( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Vector4, "set Vector4 with floats" );
    testvm( b_Vector4, "set Vector4 with floats" );
    c_Vector4 = Vector4( 0.0f );
    d_Vector4 = Vector4( 0.0f );
    e_Vector4 = Vector4( 0.0f );
    testvm( c_Vector4, "set Vector4 elements to zero" );
    testvm( d_Vector4, "set Vector4 elements to zero" );
    testvm( e_Vector4, "set Vector4 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Point3 = Point3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( Vector3( a_Point3 ), pad );
    a_Point3 = Point3( tmpV4.getXYZ( ) );
    tmpV4 = Vector4( Vector3( b_Point3 ), pad );
    b_Point3 = Point3( tmpV4.getXYZ( ) );
    testvm( a_Point3, "set Point3 with floats" );
    testvm( b_Point3, "set Point3 with floats" );
    c_Point3 = Point3( 0.0f );
    d_Point3 = Point3( 0.0f );
    e_Point3 = Point3( 0.0f );
    testvm( c_Point3, "set Point3 elements to zero" );
    testvm( d_Point3, "set Point3 elements to zero" );
    testvm( e_Point3, "set Point3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Quat = Quat( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Quat, "set Quat with floats" );
    testvm( b_Quat, "set Quat with floats" );
    c_Quat = Quat( 0.0f );
    d_Quat = Quat( 0.0f );
    e_Quat = Quat( 0.0f );
    testvm( c_Quat, "set Quat elements to zero" );
    testvm( d_Quat, "set Quat elements to zero" );
    testvm( e_Quat, "set Quat elements to zero" );
    a_Vector3 = Vector3( a_Point3 );
    testvm( a_Vector3, "construct Vector3 with Point3" );
    a_Vector3 = Vector3( randfloat() );
    testvm( a_Vector3, "set Vector3 with float" );
    a_Vector3 = Vector3( scalar_float(randfloat()) );
    testvm( a_Vector3, "set Vector3 with float" );
    aos_Vector3_0 = Vector3( 0.0f, 1.0f, 2.0f );
    aos_Vector3_1 = Vector3( 3.0f, 4.0f, 5.0f );
    aos_Vector3_2 = Vector3( 6.0f, 7.0f, 8.0f );
    aos_Vector3_3 = Vector3( 9.0f, 10.0f, 11.0f );
    testvm( aos_Vector3_3, "aos type 0" );
    testvm( aos_Vector3_2, "aos type 1" );
    testvm( aos_Vector3_1, "aos type 2" );
    testvm( aos_Vector3_0, "aos type 3" );
    testvm( select( a_Vector3, b_Vector3, 0 ), "select 0" );
    testvm( select( a_Vector3, b_Vector3, 1 ), "select 1" );
    testvm( select( a_Vector3, b_Vector3, 0 ), "select 2" );
    testvm( select( a_Vector3, b_Vector3, (unsigned int)-1 ), "select 3" );
    a_Vector3 = Vector3( xyz4[0], xyz4[1], xyz4[2] );
    testvm( a_Vector3, "load XYZ array" );
    xyz4[0] = -xyz4[0];
    xyz4[1] = -xyz4[1];
    xyz4[2] = -xyz4[2];
    xyz4[3] = -xyz4[3];
    xyz4[4] = -xyz4[4];
    xyz4[5] = -xyz4[5];
    xyz4[6] = -xyz4[6];
    xyz4[7] = -xyz4[7];
    xyz4[8] = -xyz4[8];
    xyz4[9] = -xyz4[9];
    xyz4[10] = -xyz4[10];
    xyz4[11] = -xyz4[11];
    aos_Vector4_0 = Vector4( xyz4[0], xyz4[1], xyz4[2], xyz4[3] );
    aos_Vector4_1 = Vector4( xyz4[4], xyz4[5], xyz4[6], xyz4[7] );
    aos_Vector4_2 = Vector4( xyz4[8], xyz4[9], xyz4[10], xyz4[11] );
    testvm( aos_Vector4_0, "xyzx" );
    testvm( aos_Vector4_1, "yzxy" );
    testvm( aos_Vector4_2, "zxyz" );
#ifndef _VECTORMATH_SCALAR_TEST
    loadXYZArray( aos_Vector3_0, aos_Vector3_1, aos_Vector3_2, aos_Vector3_3, (const vec_float4 *)xyz4 );
    xyz4[0] = 0;
    xyz4[1] = 1;
    xyz4[2] = 2;
    xyz4[3] = 3;
    xyz4[4] = 4;
    xyz4[5] = 5;
    xyz4[6] = 6;
    xyz4[7] = 7;
    xyz4[8] = 8;
    xyz4[9] = 9;
    xyz4[10] = 10;
    xyz4[11] = 11;
    storeXYZArray( aos_Vector3_0, aos_Vector3_1, aos_Vector3_2, aos_Vector3_3, (vec_float4 *)xyz4 );
#endif
    aos_Vector4_0 = Vector4( xyz4[0], xyz4[1], xyz4[2], xyz4[3] );
    aos_Vector4_1 = Vector4( xyz4[4], xyz4[5], xyz4[6], xyz4[7] );
    aos_Vector4_2 = Vector4( xyz4[8], xyz4[9], xyz4[10], xyz4[11] );
    testvm( aos_Vector4_0, "xyzx" );
    testvm( aos_Vector4_1, "yzxy" );
    testvm( aos_Vector4_2, "zxyz" );


	if (goldReference)
	{
		char tmpname[VM_MAX_STRING_LEN];
		char name[VM_MAX_STRING_LEN];
		sprintf(name,"storeXYZ:-1.0 -2.0 -3.0 0.4");
		char* bla = fgets(tmpname,strlen(name)+1,goldReference);
		if (strcmp(name,tmpname))
			printf("Fail: expected %s but found %s\n",name,tmpname);
		fgets(tmpname,2,goldReference);
	} else
	{
#ifdef _VECTORMATH_SCALAR_TEST
		printf("storeXYZ:-1.0 -2.0 -3.0 0.4\n");
#else
    quad = (vec_float4){-1.0f, -2.0f, -3.0f, -4.0f};
    a_Vector3 = Vector3( quad );
    quad = (vec_float4){0.1f, 0.2f, 0.3f, 0.4f};
    storeXYZ( a_Vector3, &quad );
    printf("storeXYZ:%f %f %f %f\n", ((float *)&quad)[0], ((float *)&quad)[1], ((float *)&quad)[2], ((float *)&quad)[3]);
#endif
	}

    a_Vector3 = b_Vector3;
    testvm( a_Vector3, "assign to Vector3 from Vector3" );
    a_Vector3 = Vector3( 0.0f );
    testvm( a_Vector3, "set Vector3 elements to zero" );
    a_Vector3 = Vector3::xAxis( );
    testvm( a_Vector3, "set to x axis" );
    a_Vector3 = Vector3::yAxis( );
    testvm( a_Vector3, "set to y axis" );
    a_Vector3 = Vector3::zAxis( );
    testvm( a_Vector3, "set to z axis" );
    if (iteration % 2) {
    a_Vector3.setElem( 0, randfloat() );
    } else {
    a_Vector3.setElem( 0, scalar_float(randfloat()) );
    }
    testvm( a_Vector3, "Vector3::set( 0, float )" );
    a_Vector3[0] = randfloat();
    a_Vector3[0] *= randfloat();
    a_Vector3[0] /= randfloat();
    a_Vector3[0] += randfloat();
    a_Vector3[0] -= randfloat();
    testvm( a_Vector3, "Vector3::operator [](0)" );
    a_Vector3.setX( randfloat() );
    testvm( a_Vector3, "Vector3::setX()" );
    if (iteration % 2) {
    a_Vector3.setElem( 1, randfloat() );
    } else {
    a_Vector3.setElem( 1, scalar_float(randfloat()) );
    }
    testvm( a_Vector3, "Vector3::set( 1, float )" );
    a_Vector3[1] = randfloat();
    a_Vector3[1] *= randfloat();
    a_Vector3[1] /= randfloat();
    a_Vector3[1] += randfloat();
    a_Vector3[1] -= randfloat();
    testvm( a_Vector3, "Vector3::operator [](1)" );
    a_Vector3.setY( randfloat() );
    testvm( a_Vector3, "Vector3::setY()" );
    if (iteration % 2) {
    a_Vector3.setElem( 2, randfloat() );
    } else {
    a_Vector3.setElem( 2, scalar_float(randfloat()) );
    }
    testvm( a_Vector3, "Vector3::set( 2, float )" );
    a_Vector3[2] = randfloat();
    a_Vector3[2] *= randfloat();
    a_Vector3[2] /= randfloat();
    a_Vector3[2] += randfloat();
    a_Vector3[2] -= randfloat();
    testvm( a_Vector3, "Vector3::operator [](2)" );
    a_Vector3.setZ( randfloat() );
    testvm( a_Vector3, "Vector3::setZ()" );
	
	
	testfloat("Vector3::get( 0 ): ", getfloat(a_Vector3.getElem( 0 )) );

	

    testfloat("Vector3::operator []( 0 ):", getfloat((float)a_Vector3[0]) );
    testfloat("Vector3::getX(): ", getfloat(a_Vector3.getX( )) );
    testfloat("Vector3::get( 1 ): ", getfloat(a_Vector3.getElem( 1 )) );
    testfloat("Vector3::operator []( 1 ): ", getfloat((float)a_Vector3[1]) );
    testfloat("Vector3::getY(): ", getfloat(a_Vector3.getY( )) );
    testfloat("Vector3::get( 2 ): ", getfloat(a_Vector3.getElem( 2 )) );
    testfloat("Vector3::operator []( 2 ): ", getfloat((float)a_Vector3[2]) );
    testfloat("Vector3::getZ(): ", getfloat(a_Vector3.getZ( )) );
    testvm( ( a_Vector3 + b_Vector3 ), "Vector3 + Vector3" );
    testvm( ( a_Vector3 - b_Vector3 ), "Vector3 - Vector3" );
    testvm( ( a_Vector3 + b_Point3 ), "Vector3 + Point3" );
    testvm( ( a_Vector3 * randfloat() ), "Vector3 * float" );
    testvm( ( a_Vector3 / randfloat() ), "Vector3 / float" );
    testvm( ( randfloat() * a_Vector3 ), "float * Vector3" );
    testvm( ( -a_Vector3 ), "Vector3 negate" );
    testvm( mulPerElem( a_Vector3, b_Vector3 ), "mulPerElem( Vector3, Vector3 )" );
    testvm( divPerElem( a_Vector3, b_Vector3 ), "divPerElem( Vector3, Vector3 )" );
    testvm( recipPerElem( a_Vector3 ), "Vector3 recip" );
    testvm( sqrtPerElem( absPerElem( a_Vector3 ) ), "Vector3 sqrt" );
    testvm( rsqrtPerElem( absPerElem( a_Vector3 ) ), "Vector3 rsqrt" );
    testvm( absPerElem( a_Vector3 ), "Vector3 abs" );
    testvm( copySignPerElem( a_Vector3, b_Vector3 ), "Vector3 copySign" );
    testvm( maxPerElem( a_Vector3, b_Vector3 ), "Vector3 maximum Vector3" );
    testvm( minPerElem( a_Vector3, b_Vector3 ), "Vector3 minimum Vector3" );
    testfloat("Vector3 maximum of elements: ", getfloat(maxElem( a_Vector3 )));
    testfloat("Vector3 minimum of elements: ", getfloat(minElem( a_Vector3 )));
    testfloat("Vector3 sum of elements: ", getfloat(sum( a_Vector3 )));
    testfloat("Vector3 dot Vector3: ", getfloat(dot( a_Vector3, b_Vector3 )));
    testfloat("Vector3 lengthSqr: ", getfloat(lengthSqr( a_Vector3 )));
    testfloat("Vector3 length: ", getfloat(length( a_Vector3 )));
    testvm( normalize( a_Vector3 ), "Vector3 normalized" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    e_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    b_Vector3 = normalize( b_Vector3 );
    c_Vector3 = normalize( c_Vector3 );
    d_Vector3 = normalize( d_Vector3 );
    e_Vector3 = normalize( e_Vector3 );
    a_Vector3 = lerp( randfloat(), b_Vector3, c_Vector3 );
    testvm( a_Vector3, "Vector3 lerp" );
    a_Vector3 = slerp( randfloat(), b_Vector3, c_Vector3 );
    testvm( a_Vector3, "Vector3 slerp" );
}

void
Vector4_methods_test()
{
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3, e_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4, e_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3, e_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat, e_Quat;
    Vector4 tmpV4, aos_Vector4_0, aos_Vector4_1, aos_Vector4_2, aos_Vector4_3;
    float rndflt1, rndflt2, rndflt3, rndflt4, pad;
    // set a pad value to detect invalid use of padding.
    // this will be nan for scalar/ppu implementations, max. float for spu
    union { float f; unsigned int u; } tmp;
    tmp.u = 0x7fffffff;
    pad = tmp.f;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Vector3 = Vector3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( a_Vector3, pad );
    a_Vector3 = tmpV4.getXYZ( );
    tmpV4 = Vector4( b_Vector3, pad );
    b_Vector3 = tmpV4.getXYZ( );
    testvm( a_Vector3, "set Vector3 with floats" );
    testvm( b_Vector3, "set Vector3 with floats" );
    c_Vector3 = Vector3( 0.0f );
    d_Vector3 = Vector3( 0.0f );
    e_Vector3 = Vector3( 0.0f );
    testvm( c_Vector3, "set Vector3 elements to zero" );
    testvm( d_Vector3, "set Vector3 elements to zero" );
    testvm( e_Vector3, "set Vector3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Vector4 = Vector4( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Vector4, "set Vector4 with floats" );
    testvm( b_Vector4, "set Vector4 with floats" );
    c_Vector4 = Vector4( 0.0f );
    d_Vector4 = Vector4( 0.0f );
    e_Vector4 = Vector4( 0.0f );
    testvm( c_Vector4, "set Vector4 elements to zero" );
    testvm( d_Vector4, "set Vector4 elements to zero" );
    testvm( e_Vector4, "set Vector4 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Point3 = Point3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( Vector3( a_Point3 ), pad );
    a_Point3 = Point3( tmpV4.getXYZ( ) );
    tmpV4 = Vector4( Vector3( b_Point3 ), pad );
    b_Point3 = Point3( tmpV4.getXYZ( ) );
    testvm( a_Point3, "set Point3 with floats" );
    testvm( b_Point3, "set Point3 with floats" );
    c_Point3 = Point3( 0.0f );
    d_Point3 = Point3( 0.0f );
    e_Point3 = Point3( 0.0f );
    testvm( c_Point3, "set Point3 elements to zero" );
    testvm( d_Point3, "set Point3 elements to zero" );
    testvm( e_Point3, "set Point3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Quat = Quat( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Quat, "set Quat with floats" );
    testvm( b_Quat, "set Quat with floats" );
    c_Quat = Quat( 0.0f );
    d_Quat = Quat( 0.0f );
    e_Quat = Quat( 0.0f );
    testvm( c_Quat, "set Quat elements to zero" );
    testvm( d_Quat, "set Quat elements to zero" );
    testvm( e_Quat, "set Quat elements to zero" );
    a_Vector4 = Vector4( a_Vector3, randfloat() );
    testvm( a_Vector4, "set Vector4 with Vector3, float" );
    a_Vector4 = Vector4( a_Vector3 );
    testvm( a_Vector4, "set Vector4 with Vector3" );
    a_Vector4 = Vector4( a_Point3 );
    testvm( a_Vector4, "set Vector4 with Point3" );
    a_Vector4 = Vector4( a_Quat );
    testvm( a_Vector4, "construct Vector4 with Quat" );
    a_Vector4 = Vector4( randfloat() );
    testvm( a_Vector4, "set Vector4 with float" );
    a_Vector4 = Vector4( scalar_float(randfloat()) );
    testvm( a_Vector4, "set Vector4 with float" );
    aos_Vector4_0 = Vector4( 0.0f, 1.0f, 2.0f, 3.0f );
    aos_Vector4_1 = Vector4( 4.0f, 5.0f, 6.0f, 7.0f );
    aos_Vector4_2 = Vector4( 8.0f, 9.0f, 10.0f, 11.0f );
    aos_Vector4_3 = Vector4( 12.0f, 13.0f, 14.0f, 15.0f );
    testvm( aos_Vector4_3, "aos type 0" );
    testvm( aos_Vector4_2, "aos type 1" );
    testvm( aos_Vector4_1, "aos type 2" );
    testvm( aos_Vector4_0, "aos type 3" );
    testvm( select( a_Vector4, b_Vector4, 0 ), "select 0" );
    testvm( select( a_Vector4, b_Vector4, 1 ), "select 1" );
    testvm( select( a_Vector4, b_Vector4, 0 ), "select 2" );
    testvm( select( a_Vector4, b_Vector4, (unsigned int)-1 ), "select 3" );
    a_Vector4 = b_Vector4;
    testvm( a_Vector4, "assign to Vector4 from Vector4" );
    a_Vector4.setXYZ( a_Vector3 );
    testvm( a_Vector4, "set Vector4 xyz" );
    testvm( a_Vector4.getXYZ( ), "get Vector4 xyz" );
    a_Vector4 = Vector4( 0.0f );
    testvm( a_Vector4, "set Vector4 elements to zero" );
    a_Vector4 = Vector4::xAxis( );
    testvm( a_Vector4, "set to x axis" );
    a_Vector4 = Vector4::yAxis( );
    testvm( a_Vector4, "set to y axis" );
    a_Vector4 = Vector4::zAxis( );
    testvm( a_Vector4, "set to z axis" );
    a_Vector4 = Vector4::wAxis( );
    testvm( a_Vector4, "set to w axis" );
    if (iteration % 2) {
    a_Vector4.setElem( 0, randfloat() );
    } else {
    a_Vector4.setElem( 0, scalar_float(randfloat()) );
    }
    testvm( a_Vector4, "Vector4::set( 0, float )" );
    a_Vector4[0] = randfloat();
    a_Vector4[0] *= randfloat();
    a_Vector4[0] /= randfloat();
    a_Vector4[0] += randfloat();
    a_Vector4[0] -= randfloat();
    testvm( a_Vector4, "Vector4::operator [](0)" );
    a_Vector4.setX( randfloat() );
    testvm( a_Vector4, "Vector4::setX()" );
    if (iteration % 2) {
    a_Vector4.setElem( 1, randfloat() );
    } else {
    a_Vector4.setElem( 1, scalar_float(randfloat()) );
    }
    testvm( a_Vector4, "Vector4::set( 1, float )" );
    a_Vector4[1] = randfloat();
    a_Vector4[1] *= randfloat();
    a_Vector4[1] /= randfloat();
    a_Vector4[1] += randfloat();
    a_Vector4[1] -= randfloat();
    testvm( a_Vector4, "Vector4::operator [](1)" );
    a_Vector4.setY( randfloat() );
    testvm( a_Vector4, "Vector4::setY()" );
    if (iteration % 2) {
    a_Vector4.setElem( 2, randfloat() );
    } else {
    a_Vector4.setElem( 2, scalar_float(randfloat()) );
    }
    testvm( a_Vector4, "Vector4::set( 2, float )" );
    a_Vector4[2] = randfloat();
    a_Vector4[2] *= randfloat();
    a_Vector4[2] /= randfloat();
    a_Vector4[2] += randfloat();
    a_Vector4[2] -= randfloat();
    testvm( a_Vector4, "Vector4::operator [](2)" );
    a_Vector4.setZ( randfloat() );
    testvm( a_Vector4, "Vector4::setZ()" );
    if (iteration % 2) {
    a_Vector4.setElem( 3, randfloat() );
    } else {
    a_Vector4.setElem( 3, scalar_float(randfloat()) );
    }
    testvm( a_Vector4, "Vector4::set( 3, float )" );
    a_Vector4[3] = randfloat();
    a_Vector4[3] *= randfloat();
    a_Vector4[3] /= randfloat();
    a_Vector4[3] += randfloat();
    a_Vector4[3] -= randfloat();
    testvm( a_Vector4, "Vector4::operator [](3)" );
    a_Vector4.setW( randfloat() );
    testvm( a_Vector4, "Vector4::setW()" );
    testfloat("Vector4::get( 0 ): ", getfloat(a_Vector4.getElem( 0 )) );
    testfloat("Vector4::operator []( 0 ): ", getfloat((float)a_Vector4[0]) );
    testfloat("Vector4::getX(): ", getfloat(a_Vector4.getX( )) );
    testfloat("Vector4::get( 1 ): ", getfloat(a_Vector4.getElem( 1 )) );
    testfloat("Vector4::operator []( 1 ): ", getfloat((float)a_Vector4[1]) );
    testfloat("Vector4::getY(): ", getfloat(a_Vector4.getY( )) );
    testfloat("Vector4::get( 2 ): ", getfloat(a_Vector4.getElem( 2 )) );
    testfloat("Vector4::operator []( 2 ): ", getfloat((float)a_Vector4[2]) );
    testfloat("Vector4::getZ(): ", getfloat(a_Vector4.getZ( )) );
    testfloat("Vector4::get( 3 ): ", getfloat(a_Vector4.getElem( 3 )) );
    testfloat("Vector4::operator []( 3 ): ", getfloat((float)a_Vector4[3]) );
    testfloat("Vector4::getW(): ", getfloat(a_Vector4.getW( )) );
    testvm( ( a_Vector4 + b_Vector4 ), "Vector4 + Vector4" );
    testvm( ( a_Vector4 - b_Vector4 ), "Vector4 - Vector4" );
    testvm( ( a_Vector4 * randfloat() ), "Vector4 * float" );
    testvm( ( a_Vector4 / randfloat() ), "Vector4 / float" );
    testvm( ( randfloat() * a_Vector4 ), "float * Vector4" );
    testvm( ( -a_Vector4 ), "Vector4 negate" );
    testvm( mulPerElem( a_Vector4, b_Vector4 ), "mulPerElem( Vector4, Vector4 )" );
    testvm( divPerElem( a_Vector4, b_Vector4 ), "divPerElem( Vector4, Vector4 )" );
    testvm( recipPerElem( a_Vector4 ), "Vector4 recip" );
    testvm( sqrtPerElem( absPerElem( a_Vector4 ) ), "Vector4 sqrt" );
    testvm( rsqrtPerElem( absPerElem( a_Vector4 ) ), "Vector4 rsqrt" );
    testvm( absPerElem( a_Vector4 ), "Vector4 abs" );
    testvm( copySignPerElem( a_Vector4, b_Vector4 ), "Vector4 copySign" );
    testvm( maxPerElem( a_Vector4, b_Vector4 ), "Vector4 maximum Vector4" );
    testvm( minPerElem( a_Vector4, b_Vector4 ), "Vector4 minimum Vector4" );
    testfloat("Vector4 maximum of elements: ", getfloat(maxElem( a_Vector4 )));
    testfloat("Vector4 minimum of elements: ", getfloat(minElem( a_Vector4 )));
    testfloat("Vector4 sum of elements: ", getfloat(sum( a_Vector4 )));
    testfloat("Vector4 dot Vector4: ", getfloat(dot( a_Vector4, b_Vector4 )));
    testfloat("Vector4 lengthSqr: ", getfloat(lengthSqr( a_Vector4 )));
    testfloat("Vector4 length: ", getfloat(length( a_Vector4 )));
    testvm( normalize( a_Vector4 ), "Vector4 normalized" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    e_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    b_Vector4 = normalize( b_Vector4 );
    c_Vector4 = normalize( c_Vector4 );
    d_Vector4 = normalize( d_Vector4 );
    e_Vector4 = normalize( e_Vector4 );
    a_Vector4 = lerp( randfloat(), b_Vector4, c_Vector4 );
    testvm( a_Vector4, "Vector4 lerp" );
    a_Vector4 = slerp( randfloat(), b_Vector4, c_Vector4 );
    testvm( a_Vector4, "Vector4 slerp" );
}

void
Point3_methods_test()
{
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3, e_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4, e_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3, e_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat, e_Quat;
    Vector4 tmpV4;
    Point3 aos_Point3_0, aos_Point3_1, aos_Point3_2, aos_Point3_3;
    Vector4 aos_Vector4_0, aos_Vector4_1, aos_Vector4_2;
    float rndflt1, rndflt2, rndflt3, rndflt4, pad;
    VM_ALIGNED16(float xyz4[12]);
#ifndef _VECTORMATH_SCALAR_TEST
    vec_float4 quad;
#endif
    xyz4[0] = randfloat();
    xyz4[1] = randfloat();
    xyz4[2] = randfloat();
    xyz4[3] = randfloat();
    xyz4[4] = randfloat();
    xyz4[5] = randfloat();
    xyz4[6] = randfloat();
    xyz4[7] = randfloat();
    xyz4[8] = randfloat();
    xyz4[9] = randfloat();
    xyz4[10] = randfloat();
    xyz4[11] = randfloat();
    // set a pad value to detect invalid use of padding.
    // this will be nan for scalar/ppu implementations, max. float for spu
    union { float f; unsigned int u; } tmp;
    tmp.u = 0x7fffffff;
    pad = tmp.f;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Vector3 = Vector3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( a_Vector3, pad );
    a_Vector3 = tmpV4.getXYZ( );
    tmpV4 = Vector4( b_Vector3, pad );
    b_Vector3 = tmpV4.getXYZ( );
    testvm( a_Vector3, "set Vector3 with floats" );
    testvm( b_Vector3, "set Vector3 with floats" );
    c_Vector3 = Vector3( 0.0f );
    d_Vector3 = Vector3( 0.0f );
    e_Vector3 = Vector3( 0.0f );
    testvm( c_Vector3, "set Vector3 elements to zero" );
    testvm( d_Vector3, "set Vector3 elements to zero" );
    testvm( e_Vector3, "set Vector3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Vector4 = Vector4( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Vector4, "set Vector4 with floats" );
    testvm( b_Vector4, "set Vector4 with floats" );
    c_Vector4 = Vector4( 0.0f );
    d_Vector4 = Vector4( 0.0f );
    e_Vector4 = Vector4( 0.0f );
    testvm( c_Vector4, "set Vector4 elements to zero" );
    testvm( d_Vector4, "set Vector4 elements to zero" );
    testvm( e_Vector4, "set Vector4 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Point3 = Point3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( Vector3( a_Point3 ), pad );
    a_Point3 = Point3( tmpV4.getXYZ( ) );
    tmpV4 = Vector4( Vector3( b_Point3 ), pad );
    b_Point3 = Point3( tmpV4.getXYZ( ) );
    testvm( a_Point3, "set Point3 with floats" );
    testvm( b_Point3, "set Point3 with floats" );
    c_Point3 = Point3( 0.0f );
    d_Point3 = Point3( 0.0f );
    e_Point3 = Point3( 0.0f );
    testvm( c_Point3, "set Point3 elements to zero" );
    testvm( d_Point3, "set Point3 elements to zero" );
    testvm( e_Point3, "set Point3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Quat = Quat( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Quat, "set Quat with floats" );
    testvm( b_Quat, "set Quat with floats" );
    c_Quat = Quat( 0.0f );
    d_Quat = Quat( 0.0f );
    e_Quat = Quat( 0.0f );
    testvm( c_Quat, "set Quat elements to zero" );
    testvm( d_Quat, "set Quat elements to zero" );
    testvm( e_Quat, "set Quat elements to zero" );
    a_Point3 = Point3( a_Vector3 );
    testvm( a_Point3, "construct Point3 with Vector3" );
    a_Point3 = Point3( randfloat() );
    testvm( a_Point3, "set Point3 with float" );
    a_Point3 = Point3( scalar_float(randfloat()) );
    testvm( a_Point3, "set Point3 with float" );
    aos_Point3_0 = Point3( 0.0f, 1.0f, 2.0f );
    aos_Point3_1 = Point3( 3.0f, 4.0f, 5.0f );
    aos_Point3_2 = Point3( 6.0f, 7.0f, 8.0f );
    aos_Point3_3 = Point3( 9.0f, 10.0f, 11.0f );
    testvm( aos_Point3_3, "aos type 0" );
    testvm( aos_Point3_2, "aos type 1" );
    testvm( aos_Point3_1, "aos type 2" );
    testvm( aos_Point3_0, "aos type 3" );
    testvm( select( a_Point3, b_Point3, 0 ), "select 0" );
    testvm( select( a_Point3, b_Point3, 1 ), "select 1" );
    testvm( select( a_Point3, b_Point3, 0 ), "select 2" );
    testvm( select( a_Point3, b_Point3, (unsigned int)-1 ), "select 3" );
    a_Point3 = Point3( xyz4[0], xyz4[1], xyz4[2] );
    testvm( a_Point3, "load XYZ array" );
    xyz4[0] = -xyz4[0];
    xyz4[1] = -xyz4[1];
    xyz4[2] = -xyz4[2];
    xyz4[3] = -xyz4[3];
    xyz4[4] = -xyz4[4];
    xyz4[5] = -xyz4[5];
    xyz4[6] = -xyz4[6];
    xyz4[7] = -xyz4[7];
    xyz4[8] = -xyz4[8];
    xyz4[9] = -xyz4[9];
    xyz4[10] = -xyz4[10];
    xyz4[11] = -xyz4[11];
    aos_Vector4_0 = Vector4( xyz4[0], xyz4[1], xyz4[2], xyz4[3] );
    aos_Vector4_1 = Vector4( xyz4[4], xyz4[5], xyz4[6], xyz4[7] );
    aos_Vector4_2 = Vector4( xyz4[8], xyz4[9], xyz4[10], xyz4[11] );
    testvm( aos_Vector4_0, "xyzx" );
    testvm( aos_Vector4_1, "yzxy" );
    testvm( aos_Vector4_2, "zxyz" );
#ifndef _VECTORMATH_SCALAR_TEST
    loadXYZArray( aos_Point3_0, aos_Point3_1, aos_Point3_2, aos_Point3_3, (const vec_float4 *)xyz4 );
    xyz4[0] = 0;
    xyz4[1] = 1;
    xyz4[2] = 2;
    xyz4[3] = 3;
    xyz4[4] = 4;
    xyz4[5] = 5;
    xyz4[6] = 6;
    xyz4[7] = 7;
    xyz4[8] = 8;
    xyz4[9] = 9;
    xyz4[10] = 10;
    xyz4[11] = 11;
    storeXYZArray( aos_Point3_0, aos_Point3_1, aos_Point3_2, aos_Point3_3, (vec_float4 *)xyz4 );
#endif
    aos_Vector4_0 = Vector4( xyz4[0], xyz4[1], xyz4[2], xyz4[3] );
    aos_Vector4_1 = Vector4( xyz4[4], xyz4[5], xyz4[6], xyz4[7] );
    aos_Vector4_2 = Vector4( xyz4[8], xyz4[9], xyz4[10], xyz4[11] );
    testvm( aos_Vector4_0, "xyzx" );
    testvm( aos_Vector4_1, "yzxy" );
    testvm( aos_Vector4_2, "zxyz" );

	if (goldReference)
	{
		char tmpname[VM_MAX_STRING_LEN];
		char name[VM_MAX_STRING_LEN];
		sprintf(name,"storeXYZ:-1.0 -2.0 -3.0 0.4");
		char* bla = fgets(tmpname,strlen(name)+1,goldReference);
		if (strcmp(name,tmpname))
			printf("Fail: expected %s but found %s\n",name,tmpname);
		fgets(tmpname,2,goldReference);
	} else
	{
#ifdef _VECTORMATH_SCALAR_TEST
    printf("storeXYZ:-1.0 -2.0 -3.0 0.4\n");
#else
    quad = (vec_float4){-1.0f, -2.0f, -3.0f, -4.0f};
    a_Point3 = Point3( quad );
    quad = (vec_float4){0.1f, 0.2f, 0.3f, 0.4f};
    storeXYZ( a_Point3, &quad );
    printf("storeXYZ:%f %f %f %f\n", ((float *)&quad)[0], ((float *)&quad)[1], ((float *)&quad)[2], ((float *)&quad)[3]);
#endif
	}
    a_Point3 = b_Point3;
    testvm( a_Point3, "assign to Point3 from Point3" );
    a_Point3 = Point3( 0.0f );
    testvm( a_Point3, "set Point3 elements to zero" );
    if (iteration % 2) {
    a_Point3.setElem( 0, randfloat() );
    } else {
    a_Point3.setElem( 0, scalar_float(randfloat()) );
    }
    testvm( a_Point3, "Point3::set( 0, float )" );
    a_Point3[0] = randfloat();
    a_Point3[0] *= randfloat();
    a_Point3[0] /= randfloat();
    a_Point3[0] += randfloat();
    a_Point3[0] -= randfloat();
    testvm( a_Point3, "Point3::operator [](0)" );
    a_Point3.setX( randfloat() );
    testvm( a_Point3, "Point3::setX()" );
    if (iteration % 2) {
    a_Point3.setElem( 1, randfloat() );
    } else {
    a_Point3.setElem( 1, scalar_float(randfloat()) );
    }
    testvm( a_Point3, "Point3::set( 1, float )" );
    a_Point3[1] = randfloat();
    a_Point3[1] *= randfloat();
    a_Point3[1] /= randfloat();
    a_Point3[1] += randfloat();
    a_Point3[1] -= randfloat();
    testvm( a_Point3, "Point3::operator [](1)" );
    a_Point3.setY( randfloat() );
    testvm( a_Point3, "Point3::setY()" );
    if (iteration % 2) {
    a_Point3.setElem( 2, randfloat() );
    } else {
    a_Point3.setElem( 2, scalar_float(randfloat()) );
    }
    testvm( a_Point3, "Point3::set( 2, float )" );
    a_Point3[2] = randfloat();
    a_Point3[2] *= randfloat();
    a_Point3[2] /= randfloat();
    a_Point3[2] += randfloat();
    a_Point3[2] -= randfloat();
    testvm( a_Point3, "Point3::operator [](2)" );
    a_Point3.setZ( randfloat() );
    testvm( a_Point3, "Point3::setZ()" );
    testfloat("Point3::get( 0 ): ", getfloat(a_Point3.getElem( 0 )) );
    testfloat("Point3::operator []( 0 ): ", getfloat((float)a_Point3[0]) );
    testfloat("Point3::getX(): ", getfloat(a_Point3.getX( )) );
    testfloat("Point3::get( 1 ): ", getfloat(a_Point3.getElem( 1 )) );
    testfloat("Point3::operator []( 1 ): ", getfloat((float)a_Point3[1]) );
    testfloat("Point3::getY(): ", getfloat(a_Point3.getY( )) );
    testfloat("Point3::get( 2 ): ", getfloat(a_Point3.getElem( 2 )) );
    testfloat("Point3::operator []( 2 ): ", getfloat((float)a_Point3[2]) );
    testfloat("Point3::getZ(): ", getfloat(a_Point3.getZ( )) );
    testvm( ( a_Point3 - b_Point3 ), "Point3 - Point3" );
    testvm( ( a_Point3 + b_Vector3 ), "Point3 + Vector3" );
    testvm( ( a_Point3 - b_Vector3 ), "Point3 - Vector3" );
    testvm( mulPerElem( a_Point3, b_Point3 ), "mulPerElem( Point3, Point3 )" );
    testvm( divPerElem( a_Point3, b_Point3 ), "divPerElem( Point3, Point3 )" );
    testvm( recipPerElem( a_Point3 ), "Point3 recip" );
    testvm( sqrtPerElem( absPerElem( a_Point3 ) ), "Point3 sqrt" );
    testvm( rsqrtPerElem( absPerElem( a_Point3 ) ), "Point3 rsqrt" );
    testvm( absPerElem( a_Point3 ), "Point3 abs" );
    testvm( copySignPerElem( a_Point3, b_Point3 ), "Point3 copySign" );
    testvm( maxPerElem( a_Point3, b_Point3 ), "Point3 maximum Point3" );
    testvm( minPerElem( a_Point3, b_Point3 ), "Point3 minimum Point3" );
    testfloat("Point3 maximum of elements: ", getfloat(maxElem( a_Point3 )));
    testfloat("Point3 minimum of elements: ", getfloat(minElem( a_Point3 )));
    testfloat("Point3 sum of elements: ", getfloat(sum( a_Point3 )));
    testfloat("Point projection: ", getfloat(projection( a_Point3, b_Vector3 )));
    testfloat("Point distSqrFromOrigin: ", getfloat(distSqrFromOrigin( a_Point3 )) );
    testfloat("Point distFromOrigin: ", getfloat(distFromOrigin( a_Point3 )) );
    testfloat("Point distSqr: ", getfloat(distSqr( a_Point3, b_Point3 )) );
    testfloat("Point dist: ", getfloat(dist( a_Point3, b_Point3 )) );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    e_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    a_Point3 = lerp( randfloat(), b_Point3, c_Point3 );
    testvm( a_Point3, "Point3 lerp" );
}

void
Quat_methods_test()
{
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3, e_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4, e_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3, e_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat, e_Quat;
    Vector4 tmpV4;
    Quat aos_Quat_0, aos_Quat_1, aos_Quat_2, aos_Quat_3;
    float rndflt1, rndflt2, rndflt3, rndflt4, pad;
    // set a pad value to detect invalid use of padding.
    // this will be nan for scalar/ppu implementations, max. float for spu
    union { float f; unsigned int u; } tmp;
    tmp.u = 0x7fffffff;
    pad = tmp.f;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Vector3 = Vector3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( a_Vector3, pad );
    a_Vector3 = tmpV4.getXYZ( );
    tmpV4 = Vector4( b_Vector3, pad );
    b_Vector3 = tmpV4.getXYZ( );
    testvm( a_Vector3, "set Vector3 with floats" );
    testvm( b_Vector3, "set Vector3 with floats" );
    c_Vector3 = Vector3( 0.0f );
    d_Vector3 = Vector3( 0.0f );
    e_Vector3 = Vector3( 0.0f );
    testvm( c_Vector3, "set Vector3 elements to zero" );
    testvm( d_Vector3, "set Vector3 elements to zero" );
    testvm( e_Vector3, "set Vector3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Vector4 = Vector4( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Vector4, "set Vector4 with floats" );
    testvm( b_Vector4, "set Vector4 with floats" );
    c_Vector4 = Vector4( 0.0f );
    d_Vector4 = Vector4( 0.0f );
    e_Vector4 = Vector4( 0.0f );
    testvm( c_Vector4, "set Vector4 elements to zero" );
    testvm( d_Vector4, "set Vector4 elements to zero" );
    testvm( e_Vector4, "set Vector4 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    b_Point3 = Point3( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3) );
    tmpV4 = Vector4( Vector3( a_Point3 ), pad );
    a_Point3 = Point3( tmpV4.getXYZ( ) );
    tmpV4 = Vector4( Vector3( b_Point3 ), pad );
    b_Point3 = Point3( tmpV4.getXYZ( ) );
    testvm( a_Point3, "set Point3 with floats" );
    testvm( b_Point3, "set Point3 with floats" );
    c_Point3 = Point3( 0.0f );
    d_Point3 = Point3( 0.0f );
    e_Point3 = Point3( 0.0f );
    testvm( c_Point3, "set Point3 elements to zero" );
    testvm( d_Point3, "set Point3 elements to zero" );
    testvm( e_Point3, "set Point3 elements to zero" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Quat = Quat( scalar_float(rndflt1), scalar_float(rndflt2), scalar_float(rndflt3), scalar_float(rndflt4) );
    testvm( a_Quat, "set Quat with floats" );
    testvm( b_Quat, "set Quat with floats" );
    c_Quat = Quat( 0.0f );
    d_Quat = Quat( 0.0f );
    e_Quat = Quat( 0.0f );
    testvm( c_Quat, "set Quat elements to zero" );
    testvm( d_Quat, "set Quat elements to zero" );
    testvm( e_Quat, "set Quat elements to zero" );
    a_Quat = Quat( a_Vector3, randfloat() );
    testvm( a_Quat, "set Quat with Vector3, float" );
    a_Quat = Quat( a_Vector4 );
    testvm( a_Quat, "construct Quat with Vector4" );
    a_Quat = Quat( randfloat() );
    testvm( a_Quat, "set Quat with float" );
    a_Quat = Quat( scalar_float(randfloat()) );
    testvm( a_Quat, "set Quat with float" );
    aos_Quat_0 = Quat( 0.0f, 1.0f, 2.0f, 3.0f );
    aos_Quat_1 = Quat( 4.0f, 5.0f, 6.0f, 7.0f );
    aos_Quat_2 = Quat( 8.0f, 9.0f, 10.0f, 11.0f );
    aos_Quat_3 = Quat( 12.0f, 13.0f, 14.0f, 15.0f );
    testvm( aos_Quat_3, "aos type 0" );
    testvm( aos_Quat_2, "aos type 1" );
    testvm( aos_Quat_1, "aos type 2" );
    testvm( aos_Quat_0, "aos type 3" );
    testvm( select( a_Quat, b_Quat, 0 ), "select 0" );
    testvm( select( a_Quat, b_Quat, 1 ), "select 1" );
    testvm( select( a_Quat, b_Quat, 0 ), "select 2" );
    testvm( select( a_Quat, b_Quat, (unsigned int)-1 ), "select 3" );
    a_Quat = b_Quat;
    testvm( a_Quat, "assign to Quat from Quat" );
    a_Quat.setXYZ( a_Vector3 );
    testvm( a_Quat, "set Quat xyz" );
    testvm( a_Quat.getXYZ( ), "get Quat xyz" );
    a_Quat = Quat( 0.0f );
    testvm( a_Quat, "set Quat elements to zero" );
    if (iteration % 2) {
    a_Quat.setElem( 0, randfloat() );
    } else {
    a_Quat.setElem( 0, scalar_float(randfloat()) );
    }
    testvm( a_Quat, "Quat::set( 0, float )" );
    a_Quat[0] = randfloat();
    a_Quat[0] *= randfloat();
    a_Quat[0] /= randfloat();
    a_Quat[0] += randfloat();
    a_Quat[0] -= randfloat();
    testvm( a_Quat, "Quat::operator [](0)" );
    a_Quat.setX( randfloat() );
    testvm( a_Quat, "Quat::setX()" );
    if (iteration % 2) {
    a_Quat.setElem( 1, randfloat() );
    } else {
    a_Quat.setElem( 1, scalar_float(randfloat()) );
    }
    testvm( a_Quat, "Quat::set( 1, float )" );
    a_Quat[1] = randfloat();
    a_Quat[1] *= randfloat();
    a_Quat[1] /= randfloat();
    a_Quat[1] += randfloat();
    a_Quat[1] -= randfloat();
    testvm( a_Quat, "Quat::operator [](1)" );
    a_Quat.setY( randfloat() );
    testvm( a_Quat, "Quat::setY()" );
    if (iteration % 2) {
    a_Quat.setElem( 2, randfloat() );
    } else {
    a_Quat.setElem( 2, scalar_float(randfloat()) );
    }
    testvm( a_Quat, "Quat::set( 2, float )" );
    a_Quat[2] = randfloat();
    a_Quat[2] *= randfloat();
    a_Quat[2] /= randfloat();
    a_Quat[2] += randfloat();
    a_Quat[2] -= randfloat();
    testvm( a_Quat, "Quat::operator [](2)" );
    a_Quat.setZ( randfloat() );
    testvm( a_Quat, "Quat::setZ()" );
    if (iteration % 2) {
    a_Quat.setElem( 3, randfloat() );
    } else {
    a_Quat.setElem( 3, scalar_float(randfloat()) );
    }
    testvm( a_Quat, "Quat::set( 3, float )" );
    a_Quat[3] = randfloat();
    a_Quat[3] *= randfloat();
    a_Quat[3] /= randfloat();
    a_Quat[3] += randfloat();
    a_Quat[3] -= randfloat();
    testvm( a_Quat, "Quat::operator [](3)" );
    a_Quat.setW( randfloat() );
    testvm( a_Quat, "Quat::setW()" );
    testfloat("Quat::get( 0 ): ", getfloat(a_Quat.getElem( 0 )) );
    testfloat("Quat::operator []( 0 ): ", getfloat((float)a_Quat[0]) );
    testfloat("Quat::getX(): ", getfloat(a_Quat.getX( )) );
    testfloat("Quat::get( 1 ): ", getfloat(a_Quat.getElem( 1 )) );
    testfloat("Quat::operator []( 1 ): ", getfloat((float)a_Quat[1]) );
    testfloat("Quat::getY(): ", getfloat(a_Quat.getY( )) );
    testfloat("Quat::get( 2 ): ", getfloat(a_Quat.getElem( 2 )) );
    testfloat("Quat::operator []( 2 ): ", getfloat((float)a_Quat[2]) );
    testfloat("Quat::getZ(): ", getfloat(a_Quat.getZ( )) );
    testfloat("Quat::get( 3 ): ", getfloat(a_Quat.getElem( 3 )) );
    testfloat("Quat::operator []( 3 ): ", getfloat((float)a_Quat[3]) );
    testfloat("Quat::getW(): ", getfloat(a_Quat.getW( )) );
    testvm( ( a_Quat + b_Quat ), "Quat + Quat" );
    testvm( ( a_Quat - b_Quat ), "Quat - Quat" );
    testvm( ( a_Quat * b_Quat ), "Quat * Quat" );
    testvm( ( a_Quat * randfloat() ), "Quat * float" );
    testvm( ( a_Quat / randfloat() ), "Quat / float" );
    testvm( ( randfloat() * a_Quat ), "float * Quat" );
    testvm( ( -a_Quat ), "Quat negate" );
    testfloat("Quat dot Quat: ", getfloat(dot( a_Quat, b_Quat )));
    testfloat("Quat lengthSqr: ", getfloat(norm( a_Quat )));
    testfloat("Quat length: ", getfloat(length( a_Quat )));
    testvm( normalize( a_Quat ), "Quat normalized" );
    a_Quat = Quat::identity( );
    testvm( a_Quat, "set to identity" );
    a_Quat = Quat::rotation( a_Vector3, b_Vector3 );
    testvm( a_Quat, "Quat rotation between vectors" );
    a_Quat = Quat::rotation( randfloat(), a_Vector3 );
    testvm( a_Quat, "Quat rotation axis angle" );
    a_Quat = Quat::rotationX( randfloat() );
    testvm( a_Quat, "Quat rotationX" );
    a_Quat = Quat::rotationY( randfloat() );
    testvm( a_Quat, "Quat rotationY" );
    a_Quat = Quat::rotationZ( randfloat() );
    testvm( a_Quat, "Quat rotationZ" );
    testvm( rotate( a_Quat, a_Vector3 ), "Quat rotate Vector3" );
    testvm( conj( a_Quat ), "Quat conj" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    b_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    e_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    b_Quat = normalize( b_Quat );
    c_Quat = normalize( c_Quat );
    d_Quat = normalize( d_Quat );
    e_Quat = normalize( e_Quat );
    a_Quat = lerp( randfloat(), b_Quat, c_Quat );
    testvm( a_Quat, "Quat lerp" );
    a_Quat = slerp( randfloat(), b_Quat, c_Quat );
    testvm( a_Quat, "Quat slerp" );
    a_Quat = squad( randfloat(), b_Quat, c_Quat, d_Quat, e_Quat );
    testvm( a_Quat, "Quat squad" );
}

int main()
{
	goldReference=fopen("../test1_reference.txt","r");
//    printf("\n __begin__ \n");
    for ( iteration = 0; iteration < 2; iteration++ ) {
        Vector3_methods_test();
        Vector4_methods_test();
        Point3_methods_test();
        Quat_methods_test();
    }
	if (goldReference)
	{
		fclose(goldReference);
		printf("totalPassedTests= %d\n", totalPassedTests);
		printf("totalFailedTests = %d\n", totalFailedTests);
	} else
	{
		printf("\n __end__ \n");
	}
	

    return 0;
}
