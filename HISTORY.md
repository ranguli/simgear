# History

This information is very old, but is kept for historical purposes.

## Contributions
This is an incomplete list of contributions to SimGear during its early development:

- Raul Alonzo <amil@las.es>
  Mr. Alonzo is the author of Ssystem and provided his kind permission
  for using the moon texture. I also used parts of his code as a template
  when adding the texture.  Ssystem Homepage can be found at:
  http://www1.las.es/~amil/ssystem


- Paul Bleisch <pbleisch@acm.org>
  Paul redid the "debug" system so that it would be much more flexible,
  so it could be easily disabled for production system, and so that
  messages for certain subsystems could be selectively enabled.
  Also contributed a first stab at a config file/command line parsing
  system.


- Bernie Bright <bbright@c031.aone.net.au>
  Many C++ style, usage, and implementation improvements.
  STL portability, tons o' stuff. :-)
  Currently trying to get a BeOS port together but life keeps getting
  in the way!


- Gene Buckle <geneb@deltasoft.com>
  Gene has done a lot of work getting FG to compile with the MSVC++
  compiler.  Also, he has pushed, proded, and bugged me endlessly to
  do my stuff right.  (I mean that in a good way, because sometimes
  when the little nudge in the right direction isn't working, I need
  a good hard shove.) :-)


- John Check <j4strngs@rockfish.net>
  Cloud textures


- Jean-Francois Doue
  Vector 2D, 3D, 4D and Matrix 3D and 4D inlined C++ classes.  (Based on
  Graphics Gems IV ed. Paul S. Heckbert)
  http://www.animats.com/simpleppp/ftp/public_html/topics/developers.html


- Melchior Franz
  METAR parser and fetcher. "material" animation (based on Jim Wilsons's
  "emission" animation). Debugging and extension of property listener
  features. Addition of removeChildren.


- Mathias Froehlich
  Reworked and cleaned up large parts of the infrastructure, of math
  files, animations and rendering in preparation of a transition to
  the OSG library. Added new handlers for shared and referenced objects.


- Bruce Finney <bfinney@gte.net>
  MSVC5 compatibility.


- Jean-loup Gailly and Mark Adler <zlib@gzip.org>
  Authors of the zlib library.  Used for on-the-fly compression and
  decompression routines.
  http://www.cdrom.com/pub/infozip/zlib/


- Thomas Gellekum <tg@ihf.rwth-aachen.de>
  Changes and updates for compiling on FreeBSD


- Habibie <habibie@MailandNews.com>
  RedHat package building changes for SimGear.


- Bruce Jackson of NASA <e.b.jackson@larc.nasa.gov>
  Developed the LaRCsim code under funding by NASA which we use to provide
  the flight model. Bruce has patiently answered my many, many questions.
  http://dcb.larc.nasa.gov/www/DCBStaff/ebj/ebj.html


- Maik Justus
  Fixed an old bug in the SGPropertyNode class.


- Richard Kaszeta <bofh@me.umn.edu>
  Contributed screen buffer to ppm screen shot routine.
  Rich has also helped in the early development of the Flight Gear "altitude
  hold autopilot module" by teaching Curt Olson the basics of Control Theory
  and helping him code and debug early versions. Curt's "Boss" Bob Hain
  also contributed <bob@me.umn.edu>.  Further details available at:
  http://www.menet.umn.edu/~curt/fgfs/Docs/Autopilot/AltitudeHold/AltitudeHold.html
  Rich's Homepage: http://www.menet.umn.edu/~kaszeta


- Tom Knienieder <tom@knienieder.com>
  Ported Steve's Audio library first to OpenBSD and IRIX and
  after that also to Win32


- David Megginson <david@megginson.com>
  SimGear property manager/registry


- Tim Moore
  Ported the (chrome) "shader" animation to OSG, and helped with porting
  the "material" animation.


- Curt Olson http://www.flightgear.org/~curt/
  Curt is responsible for overall project and source code management.
  He has his hands in many of the areas.


- Petter Reinholdtsen <pere@games.no>
  Incorporated the Gnu automake/autoconf system (with libtool).
  This should streamline and standardize the build process for all
  Unix-like platforms.  It should have little effect on IDE type
  environments since these don't use the Unix make system.


- Paul Schlyter <pausch@saaf.se>
  Mr. Schlyter provided Durk Talsma with all the information
  he needed to write the astro code. Mr. S. is also willing
  to answer astro-related questions whenever one needs to.
  http://welcome.to/pausch


- Durk Talsma <d.talsma@chello.nl>
  Accurate Sun, Moon, and Planets.
  Sun changes color based on position in sky.
  Moon has correct phase and blends well into the sky.
  Planets are correctly positioned and have proper magnitude.
  Added time zone support in the SGTime class.
  Help with time functions, gui, and other misc stuff.
  http://people.a2000.nl/dtals


- Mark Vallevand <Mark.Vallevand@UNISYS.com>
  Contributed some METAR parsing code.
  Contributed some win32 screen printing routines.


- Gary R. Van Sickle <tiberius@braemarinc.com>
  Contributed some initial GameGLUT support and other fixes.
  Has done some interesting preliminary work on a binary file format
  http://www.woodsoup.org/projs/ORKiD/fgfs.htm

  Has set up a 'Cygwin Tips' site that has been very helpful to many
  people in getting a Cygwin Unix-on-Windows build environment set up
  so they can build FG effectively.
  http://www.woodsoup.org/projs/ORKiD/cygwin.htm


- Norman Vine <nhv@yahoo.com>
  Provided more than uncountable URL's to the "FlightGear Community".
  Many performance optimizations throughout the code.
  Lots of windoze related contributions.
  Contributed wgs84 distance and course routines.
  Contributed a great circle route autopilot mode based on wgs84 routines.


- Ed Williams <Ed_Williams@compuserve.com>
  Contributed magnetic variation code (impliments Nima WMM 2000)
  We've also borrowed from Ed's wonderful aviation formulary at various
  times as well.
  http://www.best.com/~williams/index.html


- Jean-Claude Wippler <jcw@equi4.com>
  Author of MetaKit - a portable, embeddible database with a portable
  data file format.  This software is not GPL'd but the author is kindly
  allowing us to bundle MetaKit with our code.  MetaKit has a liberal
  X/MIT-style license.  Please see the following URL for more info:
  http://www.equi4.com/metakit


- WoodSoup Project  http://www.woodsoup.org
  Provided computing resources and services so that the Flight Gear
  project could have real home.  This includes, web services,
  ftp services, shell accounts, email lists, dns services, etc.


- Robert Allan Zeh <raz@cmg.FCNBD.COM>
  Helped me tremendously in figuring out the Cygnus win32 compiler and
  how to link with .dll's.  With out him the first runable win32
  version of FG would have been impossible.


This document was originally written by Curt L. Olson  <http://www.flightgear.org/~curt>

- 05 Jul 2000 Removed non-SimGear entries (CLO)
- 08 Mar 2000 CONTENTS RESEARCHED AND UPDATED by Oliver Delise  <delise@mail.isis.de>

## News
Version 1.9.0
* Thu Dec 18 15:12:15 CST 2008


Version 1.8.6
* Mon Dec  1 14:02:47 CST 2008


Version 1.8.5
* October 30, 2008 (source code snapshot release)


New in 0.3.10
* April 5, 2006

* Add a small accessor function to expose local timezone offset.
* Improved exception handling and made output more helpful in various places.
* Better pbuffer runtime detection.
* Add directory creation capability to file/path library.
* Added a basic reference counting class to improve robustness of
  memory management in places.  Use this for all scenegraph
  references, sgmaterial references, sgmatmodel references, and
  sgsoundsample references.
* Add support for point sprites.
* Updates to rain cone rendering.
* Add a new vector library and integrate that with improved coordinate
  system conversion code.
* Mutex locking and cleanup improvements in the threading abstraction
  library.
* Add MacOS RenderTexture support.
* Add a Nasal based io libarary that is not activated by default.
* Added a set of MS-VC8 project files.

* Various platform related bug fixes.
* Various compiler related bug/warning fixes.
* Clean up some things that triggered valgrind warnings.
* Fix a Nasal cmp() bug.
* Removed ancient version of zlib from distribution.


New in 0.3.9
* November 17, 2005

* Add support for OpenAL 1.1 (with a separate alut library.)
* Add support for volumetric shadows.  Aircraft can cast shadows on themselves
  as well as onto the ground (by Harald Johnsen.)
* New 3d volumetric clouds by Harald Johnsen (along with several rounds of
  followup fixes and improvements.)
* Remove Mark Harris's old 3d clouds because they were never properly
  integrated.  And we now have new 3d clouds.
* Add support for seasonal textures (with a set of winter textures added
  to FlightGear.)
* Updated Nasal scripting system.  Adds several new syntax convenience
  features, fixes parser bugs, fixes several internal bugs.
* Our 3d cockpit jitter problem is fixed (woohoo!)
* Add code to support rendering to a texture.
* Allow "tip" popups to pop themselves down after the appropriate
  timeout, even if the sim time is paused.
* Various low model level animation fixes and additions ... color,
  transparency, 'chrome' effects, randomized spin, etc.
* Create our own portable stdint.h implementation.
* Fixed several memory leaks.
* removeChildren() added to the property system.
* Fix many cases of 'const'.
* Fixes for cygwin, solaris/sun, Mac OS X, MSVC, gcc-3.4.x.


New in 0.3.8
* January 18, 2005

* Configure script does a sanity check for the existence of openal.
* Better pthreads detection for FreeBSD.
* Abstract out the location of gl.h, glu.h, and glut.h so we can more
  easily support MacOS which puts these in an oddball location.
* Added two new debug output types for instrumentation and systems.
* Add a name parameter to the waypoint class for supporting a route
  manager in the flightgear gps module.
* Make display list usage optional.
* Event manager: specifying a zero delay will force event execution in
  the next frame rather than entering an infinite loop.
* gcc-4.0 fix.
* Fixes to property tree loading and saving.
* Make volume inaudible at startup.
* Solaris fixes.
* For low density cloud coverage, blend the layer to nothing as we pass
  through instead of fully engulfing the aircraft in the cloud.
* Add several new capabilities to the texture management code for building
  normal maps and doing some simple on-the-fly effects on textures.
* Better error message for sound problems.
* Add support for binding a thread to a specific CPU.


New in 0.3.7
* October 12, 2004

* Add support for parsing xml from an in memory buffer, not just a file.
* Don't reduce visibility for a "clear" cloud layer.
* Add support for audio orientation (direction and cone) for internal
  view and tower view.
* Add support for drawing from display lists rather than in immediate mode.
  This provides a big performance improvement on many systems.


New in 0.3.6
* July 29, 2004

* Better MinGW support
* A bit better handling of OpenAL under Cygwin
* Switched audio system from plib's "sl/sm" to OpenAL.
* Add support for scaling an object based on distance.  The scaling
  factor is determined by a lookup table based on distance.
* Add a "flash" animation type.
* Fix cloud positioning/animation bugs.
* Fix an off-by-one error in props_io.cxx
* Clip audio gain (volume) to 1.0


New in 0.3.5
* March 26, 2004

* Added Andy's nasal interpreter for small built in scripting tasks.
  Nasal integrates nicely with FlightGear's property system.
* Added new (much simpler) metar parsing code (removed older more
  complex code.)
* Support VASI/PAPI lights correctly.
* Fixes to cloud animation.
* Updates to sky dome coloring as well as sun/moon coloring.
* Vary environment lighting with visibility (subtlety.)
* Support control of alpha test in model animation.
* Complete rewrite of the event manager.
* Updates to low level socket code to make it more flexible.
* Win32 serial port communication fixes.
* sg_geodesy rewritten to be more accurate and robust and symmetric.


New in 0.3.4
* October 22, 2003

* Removed Metakit, FlightGear no longer uses it.
* Removed all glut dependencies from SimGear.
* Moved FGEventMgr and FGSubsystemMgr over to SimGear.
* Some more incremental work on 3D clouds.
* Added some "fastmath" functions.
* Some lighting tweaks and fixes (especially for taxiways.)
* Added support for "blend" and "scale" and "texture" animations.
* Added support for animating rotations around an arbitrary axis (so the
  aircraft designer isn't forced to figure out animations as a combination
  of rotations around X, Y, and X axes.
* Updates to sky dome modeling and make cloud layers follow the curve
  of the earth (sort of.)
* Updates to sky dome, cloud, and sunrise/sunset color and lighting
  effects to make them more realistic and lifelike.
* Better support for detecting and using OpenGL extensions at run time.
* Add support for win32-pthreads in MSVC.NET
* Various MSVC fixes.
* Various Solaris fixes.
* Various cygwin/mingwin fixes.
* Various Mac OS X fixes.
* Various Irix fixes.


New in 0.3.3
* June 3, 2003

* Fix a compile problem for cygwin
* Updated/tweaked doxygen documentation in several areas


New in 0.3.2
* June 2, 2003

* Moved quite a bit of low level model management and "state"
  management code from FlightGear into SimGear and did a substantial
  amount of restructuring and dependency clean up in the process.
  Created a "scene" subdirectory with sub-subdirectories for material
  management, basic model and model animation management, sky
  rendering, and low level loaders for the "TerraGear" tile object format.
* Removed support of the flat shaded and non-textured material
  property variants.  You can still do these things, but extra states
  are no longer set up automatically.
* Removed 3d clouds from the default build ... these need a maintainer
  or better yet, a complete plib-based rewrite.
* Moved the FlightGear sound effect manager code over to SimGear.
* Updated the cloud layer surface to better follow the inverted bowl
  shape.
* Much work on cloud/sky coloring, and also much work on
  sunset/sunrise coloring.
* Fixed an obscure bug in cloud texture loading which caused each
  cloud texture to be loaded 5 times.  Ouch!
* Various class and function renaming to make them better fit into the
  standard SimGear naming scheme.
* Added some additional convenience functions to the SGPath class.
* Upgraded the distributed version of metakit.
* FreeBSD fixes.
* Irix fixes (better STL/ISO C++ header support.)
* Mingwin fixes.
* Better MacOS support
* MSVC fixes.


New in 0.3.1
* December 4, 2002

* Fix a major packaging blunder with several missing files.


New in 0.3.0
* December 3, 2002

* removed interpreter subdir
* NOMINAX fix for Cygwin/gcc-3.2
* Added some prototype 3d clouds based on Mark Harris's demos.
* Simplified the command manager interface
* Allow an "include" attribute on root PropertyList element.


New in 0.2.0
* September 6, 2002

* Modernized the autoconf/make scripts, plus lots of tweaks and enhancements.
* Removed efence support (in favor of valgrind.)

* Added a javascript interpreter.
* SGSocket reimplemented on top of plib/net libs.
* Added a new random number generation algorithm.
* Total rewrite of the strutils package.

* Patch for the random number seed.
* IA-64 w/ Intel compiler fixes.
* MSVC/MINGW fixes.
* Mac OS X fixes.
* Irix fixes.
* Code clean ups to remove warning messages.
* Optimizations in sg_binobj to reduce the amount of memory copying
  needed when loading a binobj format file.
* Fixed a couple places where variables could be used before they were
  initialized.
* Various property manager fixes and improvements.
* Fixes to cloud layer management code.
* Made the sky dome more flexible to facilitate use in other applications.


New in 0.0.18
* April 20, 2002

* Created a src/libs subdirectory for several packages that need to be
  installed by the user but are used by many other packages and may already
  be installed.  So we just bundle the source separately as a convenience
  if the user needs to build and install these.
* Upgrade to zlib-1.1.4 (security fix)
* Upgrade to metakit-2.4.2-32.tar.gz (latest upstream release)
* Added support for point objects in the scenery file format.
* Additions to the binary file format to make it *much* more flexible.
  For each major primitive type: points, triangles, fans, and strips, you
  can specify an index list of vertices, normals, colors, and texture
  coordinates.  You can skip any of these you like to save on space.
* Added support for new file features in the binary -> ascii scenery file
  decoder.
* Various code clean ups.
* Separate XML I/O operations into a separate header file.
* Major property manager rewrite, then lots of followup cleaning and
  improvements.


New in 0.0.17 (final)
* February 16, 2002

* Explicitely reset the glBlendFunc() after drawing the moon for the
  Voodoo2 linux driver since it seems to have a bug in glPopAttrib().


New in 0.0.17pre2
* February 13, 2002

* Replaced some cout's with SG_LOG's


New in 0.0.17pre1
* February 12, 2002

* Removed metakit and zlib from build process.  Tarballs are still included
  for those who's systems don't provide these packages.
* MSVC fixes.
* IRIX fixes.
* Mingwin fixes.
* Mac OS X fixes.
* FreeBSD fixes.
* Added support for Intel's C++ compiler under Linux.
* Attempt to build in support for various non-compatible automake/autoconf
  versions.
* Fix for a problem that could lead to a bogus build for people with voodoo
  cards.
* Added Norman's jpegfactory code which can create jpeg format screen shots
  of a live running application.
* implemented set/get_log_classes and set/get_log_priority.
* Socket library improvements.
* Added a C++ exception abstraction.
* Property manager fixes and improvements including node path caching
  and debug tracing of property reads/writes.
* Updates/fixes to xml handling.


New in 0.0.16
* July 12, 2001
* Various changes to the property manager implementation to better support
  dumping out the desired portions of the property tree to file.
* Don't compile the metakit demos by default (causes problems for Irix)'
* Other various tweaks for Irix.
* Added a virtual destructor to XMLVisitor to fix a warning message.
* Check for valid hostname lookup in sg_socket.cxx
* Add a function to return modified julian date without creating an
  entire SGTime class.  This avoids an extraneous warning about not
  being able to find a timezone.
* Created a mailing list for cvs commit messages.
* Fixed a potential array under/over run bug in interpolator.cxx
* Removed all references to SGValue from the property manager.  Added
  support for an "archive" bit in the property manager to control
  which values get written out.
* Added SGCommandState class so that commands can compile and save
  copies of their arguments for efficiency if they are called multiple
  times.
* Added Brian Baul's "tr" tiled rendering low level support library.
  This along with some higher level code in flightgear allows us to do
  ultra-hires tiled screen dumps suitable for large format printers.
  (Norman Vine)


New in 0.0.15
* June 20, 2001
* Various bug fixes, tweaks, and optimizations.
* Added a command manager (analogous to the property manager)
* Better Irix Mips support.
* Various gcc-3.0 fixes.
* Various MSVC fixes.
* Added MSVC project files (automatically generated from the unix
  automake configuration files.)
* Removed total size limitation on interpolation table module.
* Various Cygwin fixes.
* Added some convenience functions to point3d.
* Various compiler warning fixes.
* Added a thread wrapper class (currently only supports pthreads)
* Added IO routines for a lowlevel, native simgear binary 3d file
  format optimized for TerraGear terrain.
* Better endianness checking and support for the binary file
  read/write routines.
* Added doxygen comments for all public interface code.  Documentation
  can be accessed via the SimGear web page.
* Many FG -> SG name space changes for better consistency throughout
  this package.
* Added property aliases, repeated name tags, and a general xml
  inclusion facilities.  Many other property manager clean ups
  following a complete rewrite.
* Fixed some critical null pointer bugs in property manager.
* Magnetic variation can now be fetched for any arbitrary location.


New in 0.0.14
* December 14, 2000
* Added a module to manage waypoints and routes, calculate bearing and
  distance to next waypoint, lateral distance off route, etc.
* Moved some of the basic time management code over from flightgear.
* Bucket dimensions can be returned in meters
* Added SOCK_STREAM (TCP) socket support to the networking code.
* Updated random number generator interface so application can provide
  it's own seed value as well as use the default time seed.
* Added a routine to calculate the distance between a point and a line segment.
* Updates to the property manager and the property list loader/saver.
* Added an explanation of the rational behind our terrain texture coordinate
  generation code.
* Fixed sky dome so that the color at the horizon will always match
  the specified fog color.
* Did a fair amount of fg -> sg name space updating (lots left to do.)
* Added support for KAI C++ on linux.
* MSVC tweaks.
* MacOS tweaks.
* FreeBSD tweaks.


New in 0.0.13
* September 14, 2000
* Added support for reading and writing xml files (easyxml)
* Then updates to property manager and xml code.
* Update magnetic variation code.
* Sky code now uses glPushAttrib and glPopAttrib so it plays better with
  other ssg based apps.
* MacOS tweaks.
* MSVC tweaks.


New in 0.0.12
* July 19, 2000
* Converted project license from GPL to LGPL.
* Moved low level IO (socket, file, serial) code over to SimGear.
* Moved ephemeral code into simgear
* Moved basic world time code into simgear
* Added a property manager (registry)
* Some documentation added.


New in 0.0.11
* July 3, 2000
* Replaced gdbm with metakit
* Added drop in sky (depends on plib)
* Added spherical course and dist given two points calculations
* MSVC5 fixes
* Math/bucket/tiling tweaks from Norman Vine


New in 0.0.7
* March 29, 2000
* Added support for RedHat package building contributed by Habibie 
  <habibie@MailandNews.com>
* Added gdbm to SimGear.  Many systems will already have gdbm installed so
  it is only built if it doesn't already exist on the user's platform.
  gdbm is a set of database routines that use extendible hashing and works
  similar to the standard UNIX dbm routines.  This guarantees the availability
  of gdbm to any application that uses SimGear.
* Optimizations and bullet proofing of magnetic variation code by Norman 
  Vine and Ed Williams


New in 0.0.6
* March 27, 2000
* Added Nima World Magnetic Model 2000 contributed by Ed Williams
* Fixes for MSVC++


New in 0.0.5
* March 17, 2000
* Restructured directory layout to facilitate windows/mac IDE builds.


New in 0.0.4
* Removed mat3.h and friends (we now use plib's sg lib for these sorts of 
  things.)


New in 0.0.3
* Release that coincides with FlightGear-0.7.2

## Changelog

2010-01-17 13:14  fredb

* simgear/simgear_config.h-msvc90: Change version in comment

2010-01-17 13:05  fredb

* projects/VC90/SimGear.vcproj, simgear/simgear_config.h-msvc90:
  Update simgear config.h version

2010-01-17 13:00  fredb

* projects/VC90/SimGear.vcproj: Update MSVC 9 project

2010-01-17 12:59  fredb

* simgear/: props/AtomicChangeListener.cxx,
  scene/material/EffectBuilder.hxx, scene/tgdb/TreeBin.cxx,
  scene/util/CopyOp.cxx: Win32 fixes

2010-01-13 15:29  ehofman

* simgear/sound/soundmgr_openal.cxx: test for an AL or ALC error
  before calling an ALUT function.

2010-01-05 12:23  ehofman

* configure.ac:

  Tatsuhiro Nishioka: These patches fixes minor bug in addition to
  providing --with-openal-framework and --with-cocoa-framework.
  Now you can use your own version of OpenAL.framework for
  selecting various audio output device.  Plus, you can build FG/SG
  on Snow Leopard with cocoa configuration.

2010-01-04 20:51  torsten

* simgear/scene/tgdb/apt_signs.cxx: Csaba Halasz: Fix airport signs
  by reverting to rev 1.22 of apt_sign.cxx. Tweaked a little to
  handle missing materials better.

2010-01-04 15:53  ehofman

* simgear/sound/soundmgr_openal.cxx: MacOS returns an unsopported
  AL error when a file is not found, work around this.

2010-01-02 17:40  torsten

* simgear/math/: SGGeoc.hxx, SGGeod.hxx: add "operator == ()" to
  SGGeod and SGGeoc

2009-12-31 17:48  jmt

* simgear/math/: SGGeodesy.cxx, SGGeodesy.hxx: Allow geocentric
  distance computations to return radians.

2009-12-31 11:14  ehofman

* simgear/structure/subsystem_mgr.cxx: Also unbind subsystem groups
  in reverse order and destruct them in reverse order to be
  consistent.

2009-12-29 15:28  ehofman

* simgear/structure/subsystem_mgr.cxx: unbind in reverse order to
  try to prevent order dependency problems.

2009-12-29 10:47  ehofman

* simgear/sound/soundmgr_openal.cxx: Rearrange alut error checking
  a bit

2009-12-28 21:57  jmt

* simgear/xml/xmltok.c: XML encodings: support 'ASCII' as an alias
  for 'US-ASCII'

2009-12-26 11:07  ehofman

* simgear/sound/: soundmgr_openal.cxx, soundmgr_openal.hxx: keep a
  pointer to the OpenAL vendor and renderer for reference

2009-12-14 20:47  ehofman

* simgear/version.h.in: revert previous patch, it confuses
  FlightGear's test for detecting the proper version of SimGear

2009-12-14 20:43  ehofman

* simgear/sound/soundmgr_openal.cxx: proper typecasting

2009-12-14 10:14  ehofman

* simgear/version.h.in: make version a string

2009-12-14 07:06  timoore

* simgear/scene/material/EffectCullVisitor.cxx: don't render an
  EffectGeode if there is no valid technique

  Normal geometry has a default effect; geometry with no default
  effect is unlikely to look correct with no state set applied.
  This fixes the problem of clouds being displayed as multi-colored
  rectangles when shader effects are turned off.

  Author: Tim Moore <timoore@redhat.com> Committer: Tim Moore
  <timoore33@gmail.com>

2009-12-09 14:56  ehofman

* simgear/sound/soundmgr_openal.cxx: fix a typo.

2009-12-09 11:09  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx: Don't assign the buffer data to the sample
  in case it is a file. Test for result when calling load()

2009-12-09 10:38  ehofman

* simgear/sound/soundmgr_openal.cxx: Fix crash in
  SGSoundMgr::stop(): do not try to erase buffer items one at a
  time

2009-12-08 06:37  timoore

* simgear/scene/model/: ModelRegistry.cxx, ModelRegistry.hxx: take
  locks out of ModelRegistry

  They should be unnecessary and were causing deadlock with Effects
  that load images.

  Author: Tim Moore <timoore@redhat.com>

2009-12-06 10:56  ehofman

* simgear/sound/soundmgr_openal.cxx: add alcSuspendContext and
  alcProcessContext again to prevent sound artifacts on hardware
  accelerated soundcards.

2009-12-02 10:32  ehofman

* simgear/sound/sample_group.cxx: check if suspend, resume and
  volume changed much from the previous value before setting them.

2009-12-02 09:32  ehofman

* simgear/sound/: sample_group.cxx, soundmgr_openal.cxx: Fix
  runtime switching of sound devices.

2009-11-30 15:22  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  soundmgr_openal.cxx: Updates to allow runtime chaning of the
  sound device

2009-11-29 17:51  timoore

* configure.ac: Revert boost requirement to 1.37

  I don't want to get into testing for tr1::unordered_map, and the
  boost compatibility doesn't exist in 1.34.

  Author: Tim Moore <timoore@redhat.com>

2009-11-28 14:31  ehofman

* simgear/sound/: soundmgr_openal.cxx, soundmgr_openal.hxx: Add a
  function to retreive all available playback devices.

2009-11-28 13:59  ehofman

* simgear/sound/sample_group.cxx: initialize volume to a proper
  value

2009-11-28 11:48  ehofman

* simgear/sound/soundmgr_openal.cxx: Small bugfix

2009-11-28 11:37  ehofman

* simgear/sound/: soundmgr_openal.cxx, soundmgr_openal.hxx: Make it
  possible to specify a different device name

2009-11-26 18:24  timoore

* simgear/scene/sky/: CloudShaderGeometry.cxx,
  CloudShaderGeometry.hxx, newcloud.cxx: fix a memory leak in
  newcloud.cxx

  Author: Tim Moore <timoore@redhat.com>

2009-11-26 18:24  timoore

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx, newcloud.cxx,
  newcloud.hxx: Use an effect for 3d clouds

  Author: Tim Moore <timoore@redhat.com>

2009-11-26 18:23  timoore

* simgear/scene/material/Effect.cxx: Support for shader program
  attributes in effects

  Also, use a hash table for the effect shader program cache.

  Author: Tim Moore <timoore@redhat.com>

2009-11-26 18:23  timoore

* simgear/scene/material/: EffectBuilder.cxx, EffectBuilder.hxx,
  TextureBuilder.cxx, makeEffect.cxx: Better error reporting for
  effects

  Throw an exception when an undefine attribute value is found in
  an effect.

  Also, fix a typo in TexEnvCombine operand attributes.

  Author: Tim Moore <timoore@redhat.com>

2009-11-26 18:23  timoore

* simgear/props/props_io.cxx: add a property debugging function

  Author: Tim Moore <timoore@redhat.com>

2009-11-26 14:05  ehofman

* simgear/sound/soundmgr_openal.cxx: a slightly more readable
  version of the test

2009-11-26 13:19  ehofman

* simgear/sound/soundmgr_openal.cxx: Also test for older versions
  of OpenAL-Sample

2009-11-24 14:33  timoore

* simgear/scene/material/Effect.cxx: add depth attribute to Effect

  Author: Tim Moore <timoore@redhat.com>

2009-11-23 12:54  ehofman

* simgear/sound/soundmgr_openal.cxx: Don't forget to apply the
  doppler adjustment factor to the listener velocity also

2009-11-23 11:32  ehofman

* simgear/sound/: sample_group.cxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx: test for implementations with 'bad' doppler
  effects and adjust for it.

2009-11-23 11:31  ehofman

* simgear/sound/: openal_test2.cxx, openal_test3.cxx: fix test
  programs

2009-11-23 10:35  ehofman

* simgear/sound/: sample_group.cxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx: proper listener velocity calculation, this
  has no effect yet but is required when other models start
  emitting sounds.

2009-11-22 23:23  timoore

* acinclude.m4, configure.ac, simgear/scene/material/Effect.hxx:
  Drop required Boost version from 1.37 to 1.34

  Use boost/tr1 to bring in std::tr1::unordered_map instead of the
  Boost version.

  Author: Tim Moore <timoore@redhat.com>

2009-11-22 01:00  timoore

* simgear/scene/: material/Effect.cxx, material/Effect.hxx,
  material/makeEffect.cxx, tgdb/TreeBin.cxx: Move tree shaders to
  an effect

  Also, improve effect hash keys.

  Author: Tim Moore <timoore@redhat.com>

2009-11-22 00:59  timoore

* simgear/scene/material/Effect.cxx: Effect file support for
  GL_VERTEX_PROGRAM_TWO_SIDE and POINT_SIZE

  Author: Tim Moore <timoore@redhat.com>

2009-11-22 00:59  timoore

* simgear/scene/material/Technique.cxx: shader language predicate

  Author: Tim Moore <timoore@redhat.com>

2009-11-19 16:29  ehofman

* simgear/sound/: soundmgr_openal.cxx, sample_group.cxx: the wrong
  name also mislead me: rotate velocity to the proper quat

2009-11-18 14:49  ehofman

* simgear/sound/sample_openal.cxx: make the relative positions
  fixed against the body again.

2009-11-17 14:06  ehofman

* simgear/sound/: sample_group.cxx, soundmgr_openal.cxx,
  xmlsound.cxx: Set to default distance attenuation function but
  change the parameters a but for better sound effects (and most of
  all quieter sounds at greta distance

2009-11-17 07:19  timoore

* simgear/scene/model/SGMaterialAnimation.cxx: descend into Effects
  to find default material animation values

  Author: Tim Moore <timoore@redhat.com>

2009-11-17 07:19  timoore

* simgear/scene/: material/TextureBuilder.cxx,
  util/StateAttributeFactory.cxx, util/StateAttributeFactory.hxx:
  add a transparent texture for effects

  This is used as a default texture for the chrome animation.

  Also, fix a typo in creating combiners.

  Author: Tim Moore <timoore@redhat.com>

2009-11-17 07:19  timoore

* simgear/props/: AtomicChangeListener.cxx,
  AtomicChangeListener.hxx: Make MultiChangeListener derive
  publicly from SGPropertyChangeListener

  Otherwise it doesn't work as a listener!

  Author: Tim Moore <timoore@redhat.com>

2009-11-16 14:32  ehofman

* simgear/sound/: sample_group.cxx, sample_openal.cxx,
  sample_openal.hxx, soundmgr_openal.cxx, xmlsound.cxx: What do you
  know, the real problem turned out to be the distance attenuation
  function..

2009-11-16 00:11  timoore

* simgear/scene/model/model.cxx: When instantiating effects, copy
  user data of any created nodes.

  This bug was the cause of the huge memory consumption / death
  reported at some places: the BVH data on "raw" .ac models (random
  objects) was dropped on the floor.

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:41  timoore

* configure.ac: Bump boost version to 1.37

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:41  timoore

* simgear/: props/props.cxx, props/props.hxx,
  scene/material/Effect.cxx, scene/material/Effect.hxx,
  scene/material/makeEffect.cxx: optimize creation and sharing of
  effects

  Implement equality test and hash for for property trees.  In an
  Effect, make a hash table of Effects that inherit from it keyed
  on their unmerged property tree. Using that, makeEffect() should
  return a single Effect for given property tree description.
  Animations may change that in the future...

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:41  timoore

* projects/VC7.1/SimGear.vcproj, projects/VC90/SimGear.vcproj,
  simgear/props/AtomicChangeListener.cxx,
  simgear/props/AtomicChangeListener.hxx,
  simgear/props/ExtendedPropertyAdapter.hxx,
  simgear/props/Makefile.am, simgear/props/props.hxx,
  simgear/scene/material/Effect.cxx,
  simgear/scene/material/Effect.hxx,
  simgear/scene/material/EffectBuilder.cxx,
  simgear/scene/material/EffectBuilder.hxx,
  simgear/scene/material/EffectGeode.cxx,
  simgear/scene/material/EffectGeode.hxx,
  simgear/scene/material/TextureBuilder.cxx,
  simgear/scene/material/makeEffect.cxx,
  simgear/scene/model/SGReaderWriterXML.cxx,
  simgear/scene/model/model.cxx, simgear/scene/util/Makefile.am,
  simgear/scene/util/UpdateOnceCallback.cxx,
  simgear/scene/util/UpdateOnceCallback.hxx: Effects in models
  working for transparent materials and chrome animation

  Implementation of animated effect values via the property system.

  Add names for TexEnvCombine attributes

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:40  timoore

* simgear/scene/material/: Effect.cxx, EffectBuilder.cxx,
  EffectBuilder.hxx, TextureBuilder.cxx, TextureBuilder.hxx: Move
  Texture unit builder into TexBuilder.cxx

  Do the refactoring necessary to make that work.

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:40  timoore

* projects/VC7.1/SimGear.vcproj, projects/VC90/SimGear.vcproj,
  simgear/scene/material/Effect.cxx,
  simgear/scene/material/Effect.hxx,
  simgear/scene/material/EffectGeode.cxx,
  simgear/scene/material/Technique.cxx,
  simgear/scene/material/TextureBuilder.cxx,
  simgear/scene/material/makeEffect.cxx,
  simgear/scene/model/SGReaderWriterXML.cxx,
  simgear/scene/model/model.cxx, simgear/scene/model/model.hxx,
  simgear/scene/model/modellib.cxx, simgear/scene/util/CopyOp.cxx,
  simgear/scene/util/CopyOp.hxx, simgear/scene/util/Makefile.am:
  Effects for models

  Basically working, at last. Among other things, create effects in
    models loaded directly from .ac files; this can happen, for
  example, with the random models from the materials library.

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:40  timoore

* projects/VC7.1/SimGear.vcproj, projects/VC90/SimGear.vcproj,
  simgear/scene/util/Makefile.am,
  simgear/scene/util/SplicingVisitor.cxx,
  simgear/scene/util/SplicingVisitor.hxx: Splicing visitor for
  rewriting scene graphs with a minimum of copying

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:40  timoore

* simgear/scene/material/: Effect.cxx, Effect.hxx,
  TextureBuilder.cxx, TextureBuilder.hxx: Dump texture attributes
  of StateSet into effect property tree

  Also, decode blend function.

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:39  timoore

* simgear/: scene/material/Effect.cxx, structure/OSGUtils.hxx:
  getStateAttribute utility function

  This provides a concise, typesafe way to get attributes from
  osg::StateSet.

  Also, blew away BackRefInserter.

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:39  timoore

* simgear/scene/material/Effect.cxx: Add support for blend
  functions and alpha test functions in effects

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:39  timoore

* simgear/scene/: material/Effect.cxx, material/EffectBuilder.hxx,
  material/TextureBuilder.cxx, material/makeEffect.cxx,
  model/ModelRegistry.cxx, model/SGReaderWriterXML.cxx: wip for
  effects in models

  multi-index for effect attributes

  Author: Tim Moore <timoore@redhat.com>

2009-11-15 01:39  timoore

* simgear/scene/: model/ModelRegistry.cxx, model/ModelRegistry.hxx,
  model/SGReaderWriterXML.cxx, model/model.cxx, model/model.hxx,
  tgdb/SGReaderWriterBTG.cxx: remove CopyPolicy from ModelRegistry

  Put the responsibility for copying a loaded model directly in
  SGReaderWriterXML.

  Author: Tim Moore <timoore@redhat.com>

2009-11-12 21:41  ehofman

* simgear/sound/: sample_group.cxx, sample_openal.cxx,
  sample_openal.hxx, soundmgr_openal.cxx, soundmgr_openal.hxx:
  temporarily remove listener (viewer) and source offsets. they
  mess things up

2009-11-10 22:19  timoore

* simgear/props/props.cxx: fix typo

  Author: Tim Moore <timoore@redhat.com>

2009-11-10 15:28  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, soundmgr_openal.hxx, xmlsound.cxx: also
  recalculate the velocity in update_pos_and_orienation, so pass
  the north-east-down velocity directly and orientate position,
  orientation and velocity to OpenGL/OpenAL frames (x-right, y-up
  and z-back)

2009-11-10 09:56  timoore

* simgear/scene/: material/Effect.cxx, tgdb/userdata.cxx,
  tgdb/userdata.hxx: Move propertyExpression code from flightgear
  to simgear

  Also add a function (possibly redundant) to access the global
  property root.

  Author: Tim Moore <timoore@redhat.com>

2009-11-10 09:56  timoore

* simgear/props/: props.cxx, props.hxx, props_io.cxx: Property
  system optimizations

  Profiling startup with the new effects code exposed some
  performance gotchas. The objective is to reduce allocation of
  std::string temporaries, especially when looking up node path
  names. Also, I changed some paths to initialize strings with
  strings instead of char *; this causes less allocation, at least
  with glibc. Also, eliminate the old version of find_node and its
  helper functions by writing the template version of find_node_aux
  to handle an explicit index parameter.

  Also, add const char[] as an internal property type

  This doesn't actually add a new type to the property system, but
  allows using character arrays as arguments to certain templates.

  Author: Tim Moore <moore@blackbox.bricoworks.com> Committer: Tim
  Moore <timoore@redhat.com>

2009-11-10 09:56  timoore

* simgear/scene/: model/SGPagedLOD.cxx, model/animation.cxx,
  model/animation.hxx, util/SGSceneUserData.cxx,
  util/SGSceneUserData.hxx: Add writeLocalData functions for
  internal scenegraph classes

  This makes the scenegraph dump more complete and therefore more
  useful.

  Author: Tim Moore <timoore@redhat.com>

2009-11-09 11:28  ehofman

* simgear/sound/: xmlsound.cxx, xmlsound.hxx: allow sound effects
  in the configuration file to be added to the 'avionics' sample
  group by setting '<type>avionics</type>'.

2009-11-09 10:23  ehofman

* SimGear.dsp, SimGear.dsw: Remove old MSVC6.0 files that doen't
  work anymore

2009-11-05 17:58  ehofman

* simgear/scene/sky/sky.cxx: Oops, it was the scenery up vector,
  not the viewer up vector

2009-11-05 14:46  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Save a costly
  SGVec3d::fromGeod() calculation

2009-11-05 13:48  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: pass the geodetic position
  and view orientation to the sky reposition function

2009-11-04 11:25  ehofman

* simgear/: environment/visual_enviro.cxx, sound/sample_group.cxx:
  small fixes

2009-11-03 12:42  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx: fix directional sound
  orientation

2009-11-03 11:16  ehofman

	* simgear/sound/openal_test3.cxx: adjust to recent changes

2009-11-03 10:59  ehofman

* simgear/sound/: sample_openal.hxx, soundmgr_openal.hxx,
  xmlsound.cxx: some small fixes

2009-11-02 22:38  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx, xmlsound.cxx: Remove more unneeded code and
  properly set relative position and sound direction

2009-11-02 12:39  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx: Small code reorganization,
  mostly removing unneeded code

2009-11-02 11:31  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx: Position and orientation fixes thanks to Tim
  Moore (finaly). Code optimizations by moving code over from
  SGSoundSample to SGSampleGroup which means it will only run once
  for every sample group instead of once for every sample.

2009-11-01 18:34  ehofman

* simgear/sound/sample_openal.hxx: silently clip pitch and gain to
  their maximum values

2009-11-01 15:51  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, xmlsound.cxx: proper orientation(?) and
  comment out relative position and direction code since it messes
  with OpenAL in such a way that volume doesn't work properly
  anymore

2009-10-31 15:18  ehofman

* simgear/sound/: openal_test3.cxx, sample_group.cxx,
  sample_group.hxx, sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: another attempt at
  getting something useful without any result.

2009-10-31 09:53  ehofman

* simgear/sound/Makefile.am: fix a typo

2009-10-31 09:46  ehofman

* simgear/compiler.h: fix a gcc version test

2009-10-31 09:41  ehofman

* simgear/compiler.h: MacOS X fix

2009-10-30 13:59  ehofman

* simgear/sound/sample_openal.cxx: add relative pos back in

2009-10-29 18:03  ehofman

* simgear/sound/sample_group.cxx: pass the float pointer to the
  isNaN function instead of the SGVec3 type

2009-10-29 15:58  ehofman

* simgear/sound/sample_openal.cxx: multiply quats in the right
  order

2009-10-29 14:33  ehofman

* simgear/sound/: openal_test3.cxx, sample_group.cxx,
  sample_group.hxx, sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx, xmlsound.cxx: Commit
  the current state of affairs to see if it fixes the position code
  for others

2009-10-29 13:53  ehofman

* simgear/sound/: Makefile.am, openal_test2.cxx, openal_test3.cxx:
  another test program, using real world locations

2009-10-28 15:27  ehofman

* simgear/sound/: openal_test2.cxx, sample_group.cxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx: Fix a bug where a sample was removed from
  the sample list before it was stopped. Proper listener
  orientation when inside the airplane (need to find a good
  solution for outside view).

2009-10-27 13:21  ehofman

* simgear/sound/soundmgr_openal.cxx: also test for NaN in listener
  code

2009-10-27 13:10  ehofman

* simgear/sound/: sample_group.cxx, sample_openal.cxx,
  sample_openal.hxx: small code reorganization and addition of
  debugging tests.

2009-10-26 23:10  timoore

* simgear/scene/: material/matmodel.cxx, model/model.cxx: More
  include SGMath.hxx to keep MSVC happy

  Author: Tim Moore <timoore@redhat.com>

2009-10-26 22:06  ehofman

* simgear/sound/: sample_group.cxx, sample_openal.cxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx, xmlsound.cxx:
  FGViewer::recalcLookFrom turned out to be an excellent source of
  information for prosition and orientation

2009-10-26 11:47  ehofman

* simgear/sound/: sample_group.cxx, soundmgr_openal.cxx,
  xmlsound.cxx: re-enable sound positioning and velocity, test for
  NaN's and print a message when it happens (debugging, should be
  removed later on).

2009-10-26 10:05  ehofman

* simgear/compiler.h: move all isnan() declarations from
  sample_group.cxx to compiler.h since it's too important not to
  have available everywhere.

2009-10-24 14:57  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, soundmgr_openal.cxx: Use shared pointers for
  any reference to SGSoundSample, fix the constructor of
  SGSoundSample where data is supplied by the calling program.

2009-10-24 10:18  ehofman

* simgear/sound/: sample_group.cxx, sample_openal.cxx,
  sample_openal.hxx, soundmgr_openal.cxx: get rid of aut_ptr, it
  only works with objects that can destroyed with delete (and not
  even delete[]) which is too limited. take drastic actions to find
  the sound-not-playing-bug: set all positions and orientations to
  default all the time.

2009-10-22 14:11  ehofman

* simgear/sound/sample_openal.cxx: .. and remove some debugging
  code

2009-10-22 14:10  ehofman

* simgear/sound/: sample_openal.cxx, xmlsound.cxx: revert some test
  code

2009-10-22 14:07  ehofman

* simgear/sound/sample_openal.hxx: should use free instead of
  delete for malloced data.

2009-10-22 10:58  ehofman

* simgear/sound/: sample_group.cxx, soundmgr_openal.cxx: a few more
  temporarty debugging statements

2009-10-22 10:32  ehofman

* simgear/sound/: openal_test1.cxx, sample_group.cxx,
  sample_openal.cxx, soundmgr_openal.cxx: fix a memory leak and add
  some temporary debugging statements.

2009-10-21 18:56  ehofman

* simgear/sound/: openal_test2.cxx, sample_openal.cxx,
  xmlsound.cxx: Updates to the test utilies.

2009-10-21 18:52  ehofman

* simgear/sound/: Makefile.am, openal_test1.cxx: Updates to the
  test utilies.

2009-10-20 13:31  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: Fix a pause situation
  where more code was executed than expected. Unbind an OpenAL
  buffer from an OpenAL source when requested to stop playing.

2009-10-19 16:09  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx: use auto_ptr instead

2009-10-19 12:40  ehofman

* simgear/sound/: openal_test1.cxx, sample_group.cxx,
  sample_group.hxx, sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: do not yet add the
  relative sound position to the absolute position, it's generating
  NaN's at the moment. Fix a bunch of other small bugs

2009-10-18 20:52  ehofman

* simgear/sound/soundmgr_openal.cxx: sigh, forgot another alut* at
  the wrong place.

2009-10-18 18:31  timoore

* simgear/: io/sg_binobj.hxx, math/sg_types.hxx, misc/texcoord.cxx,
  scene/sky/oursun.cxx, scene/tgdb/SGLightBin.hxx,
  sound/sample_openal.cxx: forward declare Point3D in sg_types.hxx

  This avoids sucking in SGMath.hxx -- and therefore a bunch of OSG
  headers -- into many compilation units.

  Author: Tim Moore <timoore@redhat.com>

2009-10-18 15:55  ehofman

* simgear/sound/soundmgr_openal.cxx: restore some part of the code
  to prevent an untwanted segmentationf fault.

2009-10-18 15:44  ehofman

* simgear/sound/: sample_group.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx: Start the sound manager in a deactived
  state. This means the code now has to activate it explicitly when
  desired. A non active state means the update function will no be
  executed.

2009-10-18 11:34  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx: Don't delete the sample data if it wasn't
  constructed as a file. It's now deleted when calling free_data()
  by the owner or in the  destructor.

2009-10-18 10:48  ehofman

* simgear/structure/SGAtomic.cxx: revert to previous version

2009-10-17 15:09  ehofman

* simgear/sound/: sample_openal.cxx, soundmgr_openal.cxx: make sure
  update_late isn't executed when dt=0

2009-10-17 14:36  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx: documentation, licensing, copyright and
  small api updates.

2009-10-16 15:19  ehofman

* simgear/sound/sample_openal.cxx: convert _relative_pos to a
  vector of doubles and set the relative offset

2009-10-16 13:37  ehofman

* simgear/sound/sample_openal.hxx: _data is not an array of pointer

2009-10-16 11:45  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx: Alut <
  1.0 fixes and finaly fix the sound orientation

2009-10-15 19:08  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx:

  Initialization was done much earlier than expected resulting in
  some sample groups not being activated (and sample loading using
  OpenAL/ALUT functions to be scheduled before OpenAL was
  initilialized).

  fix alutInit counter remove left over static declaration fro
  SGSoundMgr::load

2009-10-15 14:33  ehofman

* simgear/sound/soundmgr_openal.cxx: add a debugging statement

2009-10-15 14:00  ehofman

* simgear/environment/visual_enviro.cxx: fix for latest changes

2009-10-15 11:18  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx, xmlsound.cxx: give the sample class as much
  info as possible to properly position and orientate the sounds.
  unfortunately at this time orientation seems to be from straight
  behind which means that sounds that have outer-gain set to 0.0
  will not be heard yet.

2009-10-12 19:22  ehofman

* simgear/sound/soundmgr_openal.cxx:

  Alex Buzin: I got an error with the Sunday CVS - FG crashed while
  exiting .  gdb reports SIGSEGV error at file soundmgr_openal.cxx,
  line 159.

  Error was fixed by changing lines 157-159 from:
  buffer_map_iterator buffers_current = _buffers.begin();
  buffer_map_iterator buffers_end = _buffers.end();	    for ( ;
  buffers_current != buffers_end; ++buffers_current ) { to :
    buffer_map_iterator buffers_current;
  while(_buffers.size()){	      buffers_current =
  _buffers.begin();

2009-10-11 15:38  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  sample_openal.cxx, sample_openal.hxx, soundmgr_openal.cxx,
  soundmgr_openal.hxx, xmlsound.cxx: Correct (and verrified)
  position, orientation and velocity vector. Todo: proper sound
  orientation (the all face forward using the airplane orientation
  now) and disabling doppler effect when tied to the listener

2009-10-11 10:47  ehofman

* simgear/constants.h: proper naming is everything

2009-10-09 11:00  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: OpenAL buffer
  management; add a buffer cache to prevent loading the same sample
  in memory twice. Especially useful to save memory for
  multi-aircraft configurations and (later) for AI models.

2009-10-07 14:54  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: add the option to tie a
  SampleGroup to the listener position and orientation

2009-10-07 09:31  ehofman

* simgear/sound/soundmgr_openal.cxx: alut pre-1.0 doesn't support
  aletGetError but then again, this function doesn't get called in
  that case either so just comment it out at build time

2009-10-06 22:05  jmt

* simgear/screen/: screen-dump.cxx, screen-dump.hxx: Update the
  screen-dump code to use osgDB, and hence write out files in more
  common formats (PNG, JPEG, etc). The PPM writing code is retained
  for the moment, in case someone other than FG is relying upon it.

2009-10-06 14:11  ehofman

* simgear/sound/sample_group.cxx: fix a typo

2009-10-06 14:09  ehofman

* simgear/sound/: README, sample_group.cxx, sample_openal.cxx,
  soundmgr_openal.cxx: (try to) properly align model and viewer

2009-10-05 15:42  ehofman

* simgear/sound/soundmgr_openal.cxx: default listener (master
  volume) default to 0.0 to save some ugly code in FlightGear

2009-10-05 13:10  ehofman

* simgear/sound/soundmgr_openal.hxx: add the alc.h header file vor
  OpenAL context related code

2009-10-05 10:56  ehofman

* simgear/sound/: sample_group.cxx, sample_group.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: Rename update() to
  update_late() for the sound manager to be able to initialize it
  before any other class that uses it. This will allow the
  SoundManager to be safely accessed in the constructor of those
  classes.

2009-10-04 15:52  ehofman

* simgear/: environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, sound/Makefile.am, sound/jet.wav,
  sound/openal_test2.cxx, sound/sample_group.cxx,
  sound/sample_group.hxx, sound/sample_openal.cxx,
  sound/sample_openal.hxx, sound/soundmgr_openal.cxx,
  sound/soundmgr_openal.hxx, sound/xmlsound.cxx,
  sound/xmlsound.hxx, structure/SGAtomic.cxx: Initial commit of the
  new sound system, expect more updates to follow

2009-10-02 07:44  durk

* simgear/scene/sky/: CloudShaderGeometry.cxx, newcloud.cxx: Stuart
  Buchanan:    Improvements to the cloudsystem: 	- A new xml
  format	 - Texture indexing based on the position of the
  sprite in the cloud		mass, allowing more control over
  the texture set.	    - Improved fog and shading		-
  Better sprite distribution	      - A more natural distribution
  of clouds, so no more obvious grids.

2009-10-01 00:33  timoore

* simgear/misc/: strutils.cxx, strutils.hxx: Back out
  convertToLowerCase function

  It brought in an OSG dependency; we'll just use
  boost::to_lower_copy instead.

  Author: Tim Moore <timoore@redhat.com>

2009-09-26 13:44  jmt

* simgear/misc/: strutils.cxx, strutils.hxx: Extend
  simgear::strutils with convertToLowerCase helper - currently a
  proxy for osgDB helper of the same name.

2009-09-24 09:07  ehofman

* configure.ac: just small fixes

2009-09-22 16:17  torsten

* simgear/math/SGGeodesy.cxx: Catch a possible floating point error
  in SGGeodesy::SGCartToGeod() for cartesian coordinates close to
  the geocenter region.

2009-09-19 00:38  timoore

* simgear/scene/material/EffectCullVisitor.cxx: check for null
  effect in EffectCullVisitor

  An EffectGeode might not have any effect.

  Author: Tim Moore <timoore@redhat.com>

2009-09-16 19:01  torsten

* configure.ac, simgear/io/Makefile.am, simgear/props/Makefile.am:
  Tatsuhiro Nishioka: Patches for configure.ac and Makefile.am
  files in FG/SG so Mac developers can build these in a unix way.
  These also enables Mac developers to choose either PLIB framework
  or PLIB static libs.

2009-09-16 07:06  frohlich

* simgear/math/: SGVec2.hxx, SGVec3.hxx, SGVec4.hxx: Correct finite
  precision issues.  Use consistent function names.  Implement
  changes consistently over the different vector sizes.

  Modified Files:	  SGVec2.hxx SGVec3.hxx SGVec4.hxx

2009-09-16 07:04  frohlich

* simgear/structure/: SGSharedPtr.hxx, SGWeakPtr.hxx: Make the weak
  pointer work. Some bits were left when importing.

  Modified Files:	  SGSharedPtr.hxx SGWeakPtr.hxx

2009-09-14 14:36  jmt

* simgear/sound/xmlsound.cxx: Logging:less verbose sound loading.

2009-09-14 14:30  jmt

* simgear/scene/tgdb/TileEntry.cxx: Logging: quiet down STG
  parsing.

2009-09-14 14:20  jmt

* simgear/scene/model/ModelRegistry.cxx: Logging: quiet down
  model/image loading policy.

2009-09-14 14:18  jmt

* simgear/scene/model/ModelRegistry.cxx: Logging: quiet the BVH
  building policy.

2009-09-14 14:09  jmt

* simgear/scene/material/matlib.cxx: Logging: quiet material
  loading.

2009-09-14 14:08  jmt

* simgear/sound/xmlsound.cxx: Logging - downgrade play/stop
  messages to debug.

2009-09-09 23:32  timoore

* simgear/math/: SGVec3.hxx, vector.cxx, vector.hxx: Implement
  vector _projection_ functions.

2009-09-09 23:30  timoore

* simgear/scene/model/animation.cxx: Make "repeat" start slower on
  pick-animation mouse events;

  otherwise it is unusable.

2009-09-08 15:50  ehofman

* simgear/sound/: soundmgr_openal.cxx, xmlsound.cxx: Dont execute
  code in case the soundmanager isn't properly initialized

2009-09-07 23:42  frohlich

* simgear/: math/SGGeod.cxx, math/SGQuat.hxx, math/SGVec2.hxx,
  math/SGVec3.hxx, math/SGVec4.hxx, scene/material/Effect.cxx,
  scene/model/SGInteractionAnimation.cxx,
  scene/model/SGMaterialAnimation.cxx,
  scene/model/SGRotateTransform.cxx,
  scene/model/SGScaleTransform.cxx,
  scene/model/SGTranslateTransform.cxx, scene/model/animation.cxx,
  scene/model/particles.cxx, scene/model/placement.cxx,
  scene/model/shadanim.cxx, scene/sky/CloudShaderGeometry.cxx,
  scene/sky/cloud.cxx, scene/sky/cloudfield.cxx,
  scene/sky/dome.cxx, scene/sky/sky.cxx,
  scene/tgdb/GroundLightManager.cxx, scene/tgdb/SGOceanTile.cxx,
  scene/tgdb/SGTexturedTriangleBin.hxx,
  scene/tgdb/SGVasiDrawable.cxx, scene/tgdb/TreeBin.cxx,
  scene/tgdb/obj.cxx, scene/tgdb/pt_lights.cxx,
  scene/util/SGUpdateVisitor.hxx: Switch to new vector conversion
  functions.

  Modified Files:	  simgear/math/SGGeod.cxx
  simgear/math/SGQuat.hxx	  simgear/math/SGVec2.hxx
  simgear/math/SGVec3.hxx	  simgear/math/SGVec4.hxx
  simgear/scene/material/Effect.cxx
  simgear/scene/model/SGInteractionAnimation.cxx
  simgear/scene/model/SGMaterialAnimation.cxx
  simgear/scene/model/SGRotateTransform.cxx
  simgear/scene/model/SGScaleTransform.cxx
  simgear/scene/model/SGTranslateTransform.cxx
  simgear/scene/model/animation.cxx
  simgear/scene/model/particles.cxx
  simgear/scene/model/placement.cxx
  simgear/scene/model/shadanim.cxx
  simgear/scene/sky/CloudShaderGeometry.cxx
  simgear/scene/sky/cloud.cxx simgear/scene/sky/cloudfield.cxx
  simgear/scene/sky/dome.cxx simgear/scene/sky/sky.cxx
  simgear/scene/tgdb/GroundLightManager.cxx
  simgear/scene/tgdb/SGOceanTile.cxx
  simgear/scene/tgdb/SGTexturedTriangleBin.hxx
  simgear/scene/tgdb/SGVasiDrawable.cxx
  simgear/scene/tgdb/TreeBin.cxx simgear/scene/tgdb/obj.cxx
  simgear/scene/tgdb/pt_lights.cxx
  simgear/scene/util/SGUpdateVisitor.hxx

2009-09-07 22:37  frohlich

* simgear/scene/: material/matlib.cxx, material/matlib.hxx,
  model/BoundingVolumeBuildVisitor.hxx: Fix problem with ocean
  files not recognized as water.  Move dynamic casts to EffectGeode
  into the findMaterial method.

  Modified Files:	  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx
  simgear/scene/model/BoundingVolumeBuildVisitor.hxx

2009-09-05 14:25  frohlich

* simgear/math/SGQuat.hxx: Add some comments.  Make sure floating
  point constants do not introduce useless upcasts.  Remove now
  unused and not really usefull method.

  Modified Files:	  simgear/math/SGQuat.hxx

2009-09-05 13:56  ehofman

* simgear/magvar/Makefile.am: Also install coremag.hxx since it is
  being used by JSBSim now

2009-09-05 08:54  frohlich

* simgear/scene/util/SGSceneFeatures.hxx: Remove unused member.

  Modified Files:	  simgear/scene/util/SGSceneFeatures.hxx

2009-09-05 08:53  frohlich

* simgear/: scene/sky/dome.cxx, math/SGGeod.cxx, math/SGGeod.hxx,
  math/SGQuat.hxx, math/SGVec2.hxx, math/SGVec3.hxx,
  math/SGVec4.hxx: Should be now more easy to make use of SGMath
  without having osg.

  Modified Files:	  simgear/scene/sky/dome.cxx
  simgear/math/SGGeod.cxx      simgear/math/SGGeod.hxx
  simgear/math/SGQuat.hxx	  simgear/math/SGVec2.hxx
  simgear/math/SGVec3.hxx	  simgear/math/SGVec4.hxx

2009-09-03 22:14  frohlich

* simgear/scene/model/SGMaterialAnimation.cxx: Return a osg::Vec
  value instead of a non const reference.

  Modified Files:	  scene/model/SGMaterialAnimation.cxx

2009-09-03 22:13  frohlich

* simgear/scene/model/: SGTranslateTransform.cxx,
  SGRotateTransform.cxx, SGScaleTransform.cxx: Avoid the non const
  SGVec*::osg() method.

  Modified Files:	  SGTranslateTransform.cxx
  SGScaleTransform.cxx	 SGRotateTransform.cxx

2009-09-03 22:10  frohlich

* simgear/scene/model/: CheckSceneryVisitor.cxx,
  CheckSceneryVisitor.hxx: Use const refs for const data.

  Modified Files:
  simgear/scene/model/CheckSceneryVisitor.hxx
  simgear/scene/model/CheckSceneryVisitor.cxx

2009-08-29 15:38  torsten

* simgear/scene/model/: SGText.cxx, SGText.hxx: Spare one node in
  the scenegraph if there is no  <offsets>

2009-08-25 09:19  timoore

* simgear/scene/: material/Effect.cxx, tgdb/ShaderGeometry.cxx,
  tgdb/ShaderGeometry.hxx, tgdb/TreeBin.cxx, tgdb/TreeBin.hxx,
  tgdb/obj.cxx: Change trees code to use a faster OpenGL path

  The (random) dimensions of a large number of trees is stored in
  an array shared by all the tree geodes. The coordinates of the
  origin of each tree are replicated in an another array. This
  allows an entire block of trees to be rendered with a few OpenGL
  calls, instead of one function call per tree.

2009-08-24 19:30  torsten

* projects/VC90/SimGear.vcproj: support osgText in models

2009-08-24 19:29  torsten

* simgear/scene/model/: Makefile.am, SGReaderWriterXML.cxx,
  SGText.cxx, SGText.hxx: support osgText in models. See
  docs/README.osgtext for details

2009-08-21 17:55  torsten

* simgear/scene/tgdb/apt_signs.cxx: Don't load materials without a
  name

2009-08-21 14:10  torsten

* simgear/math/SGGeodesy.cxx: Avoid NAN due to floating point
  rounding errors

2009-08-21 09:43  ehofman

* simgear/scene/sky/oursun.cxx: prevent division by zero

2009-08-20 17:24  torsten

* simgear/screen/: RenderTexture.cpp, TestRenderTexture.cpp:  Alan
  Teeder: fix incomplete dbg_printf for non-gcc.

2009-08-20 15:10  torsten

* simgear/: misc/Makefile.am, sound/Makefile.am: don't build
  tabbed_value_test, swap_test, openal_test1 and openal_test2 by
  default.

2009-08-20 15:09  torsten

* simgear/sound/openal_test1.cxx: warning fix: unused variables

2009-08-20 15:09  torsten

* simgear/screen/TestRenderTexture.cpp: extinguish many warnings
  (at least for gcc)

2009-08-20 15:08  torsten

* simgear/scene/: model/CheckSceneryVisitor.cxx,
  sky/cloudfield.cxx, tgdb/TileEntry.cxx, tgdb/TreeBin.cxx: warning
  fix: initializing members in the order they are declared keeps
  gcc happy

2009-08-20 13:53  torsten

* simgear/scene/sky/: CloudShaderGeometry.cxx, newcloud.cxx:
  warning fixes

2009-08-20 13:44  torsten

* simgear/screen/RenderTexture.cpp: extinguish many warnings (at
  least for gcc)

2009-08-20 13:09  torsten

* simgear/scene/util/PrimitiveUtils.cxx: warning fix: abort program
  and spit out a message if getNumPrims() is called with unknown
  mode. (Shouldn't happen anyway)

2009-08-20 13:02  torsten

* simgear/scene/sky/cloud.cxx: warning fix: unused variable

2009-08-20 13:02  torsten

* simgear/scene/model/SGClipGroup.cxx: warning fix, unused code

2009-08-20 12:55  torsten

* simgear/scene/tgdb/apt_signs.cxx: don't use uninitialized
  variables

2009-08-20 12:55  torsten

* simgear/scene/tgdb/: SGTexturedTriangleBin.hxx,
  ShaderGeometry.cxx: warning fixes

2009-08-20 11:17  torsten

* simgear/scene/model/: ModelRegistry.cxx, animation.cxx,
  ModelRegistry.hxx, shadanim.cxx: warning fixes

2009-08-20 11:00  torsten

* simgear/scene/material/: Effect.cxx, makeEffect.cxx: warning
  fixes

2009-08-20 10:51  torsten

* simgear/math/SGIntersect.hxx: warning fix (multiline comment)

2009-08-20 10:46  torsten

* simgear/props/: props.cxx, props.hxx, props_io.cxx: warning fix

2009-08-20 10:43  torsten

* simgear/environment/visual_enviro.cxx: warning fix

2009-08-20 10:41  torsten

* simgear/math/SGGeodesy.cxx: warning fixes

2009-08-20 10:32  torsten

* simgear/structure/exception.cxx: Warning fix: array subscript is
  above array bounds

2009-08-16 21:33  ehofman

* simgear/scene/sky/: oursun.cxx, oursun.hxx, sky.hxx: Expose the
  color of the sun (which is not the scene specular color anymore)

2009-08-16 10:42  ehofman

* simgear/scene/sky/: oursun.cxx, oursun.hxx: Differentiate between
  sun color (based in visibility) and scene color (based on
  humidity)

2009-08-09 12:49  torsten

* simgear/scene/: material/mat.cxx, material/mat.hxx, tgdb/obj.cxx,
  tgdb/SGTexturedTriangleBin.hxx:  Stuart Buchanan: I've been
  working on a small patch to allow trees to be grouped together
  into woods. This allows what seems to me to be a more realistic
  grouping of trees for farmland in particular.

2009-08-08 14:26  fredb

* projects/VC90/SimGear.vcproj,
  simgear/scene/material/EffectBuilder.cxx,
  simgear/scene/material/TextureBuilder.cxx: Compile under MSVC9

2009-08-08 12:19  timoore

* projects/VC7.1/SimGear.vcproj, simgear/scene/material/Effect.cxx,
  simgear/scene/material/EffectBuilder.cxx,
  simgear/scene/material/EffectBuilder.hxx,
  simgear/scene/material/Makefile.am,
  simgear/scene/material/Noise.cxx,
  simgear/scene/material/Noise.hxx,
  simgear/scene/material/TextureBuilder.cxx,
  simgear/scene/material/TextureBuilder.hxx,
  simgear/scene/material/makeEffect.cxx,
  simgear/scene/material/mat.cxx, simgear/scene/material/mat.hxx:
  New effects from Till Busch: crops, water, landmass

  As shown at LinuxTag, with modifications from Tim Moore: the base
  landmass texture is mixed with the steepness and snow effects.
  Till's new syntax for textures in effect files was also added.
  syntax for textures. Also, syntax for accessing internal
  textures, such as Till's 3D noise texture, was added.

  Several bugs in the effect inheritance algorithm were fixed.

2009-07-28 16:33  torsten

* simgear/scene/model/animation.cxx: Allow multiple <button>
  elements for <action> elements in pick animations. Nice to have
  to have a single action  for mouse-button and mouse-wheel.

2009-07-27 11:50  fredb

* simgear/: math/interpolater.cxx, misc/interpolator.cxx,
  props/condition.cxx, props/props.hxx,
  scene/bvh/BVHLineSegmentVisitor.cxx,
  scene/model/CheckSceneryVisitor.cxx, scene/model/SGPagedLOD.cxx,
  scene/model/modellib.cxx, scene/model/persparam.cxx,
  structure/SGBinding.cxx, structure/commands.cxx,
  structure/event_mgr.cxx, structure/subsystem_mgr.cxx: Fix case
  typo

2009-07-26 22:24  fredb

* simgear/: props/props.hxx, scene/material/Technique.cxx,
  structure/SGExpression.hxx: Suppress warnings

2009-07-26 20:53  fredb

* projects/VC7.1/SimGear.vcproj, projects/VC90/SimGear.vcproj,
  simgear/math/interpolater.cxx, simgear/misc/interpolator.cxx,
  simgear/props/condition.cxx, simgear/props/props.cxx,
  simgear/scene/bvh/BVHLineSegmentVisitor.cxx,
  simgear/scene/material/Effect.cxx,
  simgear/scene/material/EffectCullVisitor.cxx,
  simgear/scene/material/EffectGeode.cxx,
  simgear/scene/material/Technique.cxx,
  simgear/scene/material/makeEffect.cxx,
  simgear/scene/model/CheckSceneryVisitor.cxx,
  simgear/scene/model/SGPagedLOD.cxx,
  simgear/scene/model/modellib.cxx,
  simgear/scene/model/persparam.cxx,
  simgear/structure/SGBinding.cxx, simgear/structure/commands.cxx,
  simgear/structure/event_mgr.cxx,
  simgear/structure/subsystem_mgr.cxx: Compile latest SimGear under
  MSVC9

2009-07-19 23:05  timoore

* simgear/structure/: SGExpression.cxx, SGExpression.hxx: Create a
  singleton for the parser table

2009-07-19 22:40  timoore

* simgear/: props/props.cxx, scene/material/Effect.cxx: Move
  definition of SGRawBase<...>::printOn out of simgear namespace

  They are declared in the global namespace, and MSVC insists that
  they be defined there.

2009-07-18 10:13  timoore

* simgear/scene/material/: Effect.cxx, Technique.hxx: Add missing
  return statements

2009-07-17 16:21  timoore

* simgear/scene/material/mat.cxx: Store material data with Effect

  Somehow this got left out, which broke ground intersection
  queries.

2009-07-17 15:21  timoore

* simgear/scene/material/matlib.cxx: include file change to make
  simgear work with OSG 2.9.X again

2009-07-17 14:58  timoore

* simgear/: props/condition.cxx, props/props.cxx,
  props/props_io.cxx, scene/material/Effect.cxx: Change references
  to property types

  BOOL, FLOAT etc. conflict with typedefs in windows.h, so these
  types are referred to using the props:: namespace.

2009-07-17 12:11  timoore

* simgear/props/: props.cxx, props.hxx: Changed
  SGRawValue::DefaultValue to an inline function.

  This avoids MSVC bugs in declaring templated specializations of
  static members.

2009-07-17 12:11  timoore

* simgear/: scene/material/Technique.hxx, structure/SGAtomic.hxx:
  Use SGAtomic's compareAndExchange instead of a new SGSwappable
  class

  Also, eliminate the __declspec(32) of that class which is causing
  problems in osg::buffered_object.

2009-07-16 18:35  timoore

* simgear/scene/material/: Effect.cxx, makeEffect.cxx, mat.cxx,
  mat.hxx, matlib.cxx: Fix effects code to work with OSG 2.8.2-rc4

2009-07-16 13:04  timoore

* simgear/scene/material/: Effect.cxx, Technique.cxx: Use
  std::back_inserter instead of my local hack

  BackRefInsertIterator is probably broken and may not be needed at
  all.

2009-07-16 01:12  timoore

* projects/VC7.1/SimGear.vcproj: Add new Effects files to vcproj

2009-07-16 01:11  timoore

* simgear/: scene/material/Effect.cxx,
  scene/material/Technique.cxx, scene/material/Technique.hxx,
  structure/SGExpression.cxx, structure/SGExpression.hxx: Fixes for
  technique predicates

  Get tests based on properties and OpenGL extensions working.

2009-07-16 01:11  timoore

* simgear/scene/material/: Effect.cxx, mat.cxx, mat.hxx: more
  effects features

  Materials can specify an effect.

  Add support for PolygonMode and initial support for Uniform.

2009-07-16 01:10  timoore

* simgear/scene/material/: Effect.cxx, Effect.hxx, Makefile.am,
  Pass.cxx, Technique.cxx, makeEffect.cxx, mat.cxx, mat.hxx,
  matlib.cxx: Construct effects from property lists

  The material code constructs a property list from its input
  parameters.

  Enable dumping of Pass and Technique objects to a file.

  Default effect now uses texture node instead of texture0

2009-07-16 01:10  timoore

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  material/matlib.cxx, material/matlib.hxx,
  model/BoundingVolumeBuildVisitor.hxx, tgdb/SGOceanTile.cxx,
  tgdb/apt_signs.cxx: materials use only simgear::Effect

  Eliminate SGMaterial::get_state function.

  Use Effect in BVH visitor, ocean tile generation, and airport
  signs.

2009-07-16 01:10  timoore

* simgear/scene/: material/EffectCullVisitor.cxx, material/mat.cxx,
  tgdb/obj.cxx, tgdb/pt_lights.cxx, tgdb/pt_lights.hxx: Use Effect
  to implement point lights

  This allows different OpenGL features (point sprites, point
  attenuation) to be used depending on hardware support.

2009-07-16 01:10  timoore

* simgear/scene/material/: Technique.cxx, Technique.hxx, mat.cxx:
  Use SGExpressions for evaluating a Technique's validity

2009-07-16 01:10  timoore

* simgear/: scene/model/animation.cxx, structure/SGExpression.cxx,
  structure/SGExpression.hxx: Overhaul of SGExpression

  Polymorphic additions to expressions: Add an expression base
  class with a method for dynamically determining the type of an
  expression.

  Add variables, predicates and boolian expressions.

  Support for parsing trees of expressions

2009-07-16 01:09  timoore

* simgear/: scene/material/Effect.cxx,
  scene/material/GLPredicate.cxx, scene/material/GLPredicate.hxx,
  scene/material/Makefile.am, scene/material/Pass.cxx,
  scene/material/Pass.hxx, scene/material/Technique.cxx,
  scene/material/mat.cxx, scene/sky/cloud.cxx,
  structure/Makefile.am, structure/StringTable.cxx,
  structure/StringTable.hxx, structure/intern.cxx,
  structure/intern.hxx: Work in progress for Technique validation

2009-07-16 01:09  timoore

* simgear/scene/: material/Effect.cxx, material/Effect.hxx,
  material/EffectCullVisitor.cxx, material/EffectCullVisitor.hxx,
  material/EffectGeode.cxx, material/EffectGeode.hxx,
  material/Makefile.am, material/Pass.cxx, material/Pass.hxx,
  material/Technique.cxx, material/Technique.hxx, material/mat.cxx,
  material/mat.hxx, material/matlib.cxx, sky/cloud.cxx,
  tgdb/obj.cxx: Use Effects in materials library, and therefore in
  scenery

2009-07-16 01:09  timoore

* simgear/: scene/material/Effect.cxx, scene/material/Effect.hxx,
  scene/material/EffectData.cxx, scene/material/EffectData.hxx,
  scene/material/EffectElement.hxx,
  scene/material/EffectElementBuilder.hxx,
  scene/material/EffectGeode.cxx, scene/material/EffectGeode.hxx,
  scene/material/ElementBuilder.cxx,
  scene/material/ElementBuilder.hxx, scene/material/Makefile.am,
  scene/material/Pass.cxx, scene/material/Pass.hxx,
  scene/material/Technique.cxx, scene/material/Technique.hxx,
  structure/SGAtomic.hxx, xml/XMLStaticParser.hxx: Effects
  framework

2009-07-16 01:08  timoore

* simgear/scene/material/: matlib.cxx, matlib.hxx: Blow away unused
  SGMaterialLib::add_item functions

2009-07-16 01:08  timoore

* simgear/: scene/model/model.hxx, structure/SGSharedPtr.hxx:
  cleanup

  Add support for boost::mem_fn to SGSharedPtr.

  Remove a couple of "using" declarations from
  scene/model/model.hxx

2009-07-16 01:08  timoore

* simgear/props/: props.cxx, props.hxx: Add PropertyList typedef
  for vectors of property list nodes.

2009-07-16 01:08  timoore

* simgear/props/props.hxx: Add a method to setStringValue that
  takes a std::string argument

2009-07-16 01:08  timoore

* simgear/props/: props.cxx, props.hxx: Don't cache results of
  getDisplayName

  Return a std::string result instead of char *.

2009-07-16 01:08  timoore

* simgear/props/: props.cxx, props.hxx, props_io.cxx, props_io.hxx:
  Add VEC3D and VEC4D property types

  Add "extended" argument to readProperties, which controls whether
  the vector  property types are accepted by the XML reader.

2009-07-16 01:07  timoore

* simgear/props/: props.cxx, props.hxx: Extend properties to
  support new property types.

  An SGRawBase class has been added as a base class to the
  SGRawValue hierarchy so that SGPropertyValue functions don't
  necessarily need to know the type of the value stored in the
  node.

  A new SGRawValueContainer class stores properties that shouldn't
  be stored in the node itself. PropertyTraits indicates if a type
  is stored in the property node or externally.

  Add getValue and SetValue template member functions to
  SGPropertyNode.

  Read and write new extended properties.

  Rearrange props.hxx a bit so that the template magic actually
  works.

  Split out extended raw value virtual functions into a separate
  base class.

  SGRawExtended is chosen as a base class of SGRawValue for
  extended property types.

2009-07-16 01:07  timoore

* simgear/props/: condition.cxx, props.cxx, props.hxx,
  props_io.cxx: Cleanup of properties

  Change most uses of the SGPropertyNode _value union to use
  static_cast.

  Move SGPropertyNode::Type out of the class into simgear::props
  namespace. Add a PropertyTraits class so that templates can
  calculate the property type tag based on a C++ type.

  In destructor, delete _value.val if it is not 0 (and the property
  is not aliased).

2009-07-12 19:48  fredb

* simgear/screen/RenderTexture.cpp: Refactor GLX only debug code

2009-07-07 00:58  fredb

* simgear/screen/RenderTexture.cpp: Compile when not GLX

2009-07-02 16:01  ehofman

* simgear/screen/: RenderTexture.cpp, TestRenderTexture.cpp:

  Geoff McLane: add a bunch of debugging code to find a persistant
  bug for Atlas/Map and fix them one by one.

2009-06-29 00:20  fredb

* simgear/nasal/naref.h: Support MSVC 64-bit architecture

2009-06-28 22:27  fredb

* projects/VC90/: .cvsignore, SimGear.vcproj: Add MSVC90 (VS2008)
  project files

2009-06-28 15:34  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects - Put
  PLIB and FLTK in 3rdParty

2009-06-28 11:19  frohlich

* simgear/scene/tgdb/: TileCache.cxx, TileEntry.cxx, TileEntry.hxx:
  Remove unneeded explicit scenegraph deletion.

  Modified Files:	  simgear/scene/tgdb/TileCache.cxx
  simgear/scene/tgdb/TileEntry.cxx
  simgear/scene/tgdb/TileEntry.hxx

2009-06-27 09:41  fredb

* simgear/scene/tgdb/TreeBin.cxx: Suppress warnings

2009-06-27 08:41  fredb

* simgear/scene/bvh/: BVHGroup.cxx, BVHTransform.cxx: Win32 fix

2009-06-24 07:19  frohlich

* simgear/structure/: Makefile.am, SGAtomic.cxx, SGAtomic.hxx,
  SGReferenced.hxx, SGSharedPtr.hxx, SGWeakPtr.hxx,
  SGWeakReferenced.hxx: Provide a thread safe SGWeakPtr
  implementation.  Extend SGAtomic with atomic exchange and add.
  Import updates from the original implementation of that in
  OpenFDM.

  Modified Files:	  Makefile.am SGAtomic.cxx SGAtomic.hxx
  SGReferenced.hxx	    SGSharedPtr.hxx Added Files:
  SGWeakPtr.hxx SGWeakReferenced.hxx

2009-06-23 22:35  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2009-06-23 22:35  fredb

* simgear/scene/model/SGInteractionAnimation.cxx: Win32 fix

2009-06-23 22:34  frohlich

* simgear/scene/model/ModelRegistry.cxx: No observed_ptr needed.

  Modified Files:	  simgear/scene/model/ModelRegistry.cxx

2009-06-23 22:26  frohlich

* simgear/scene/tgdb/TileEntry.cxx: Remove unused headers.

  Modified Files:	  simgear/scene/tgdb/TileEntry.cxx

2009-06-23 22:26  frohlich

* simgear/scene/model/SGReaderWriterXML.cxx: Propagate the lanel
  loader through the options.

  Modified Files:	  simgear/scene/model/SGReaderWriterXML.cxx

2009-06-22 20:39  frohlich

* simgear/scene/model/: SGInteractionAnimation.hxx,
  SGInteractionAnimation.cxx, Makefile.am, animation.cxx: Move the
  carrier interactive geometry configuration into the model files.

  Modified Files:	  Makefile.am animation.cxx Added Files:
  SGInteractionAnimation.hxx SGInteractionAnimation.cxx

2009-06-22 11:35  ehofman

* simgear/screen/: Makefile.am, colours.h, texture.cxx,
  texture.hxx: Move the texture code to FlightGear/utils/Modeller

2009-06-20 13:11  frohlich

* simgear/math/SGIntersect.hxx: Add some trailing spaces ...

  Modified Files:	  ./simgear/math/SGIntersect.hxx

2009-06-20 11:33  jmt

* simgear/props/condition.cxx: Fix a warning from GCC - 'ALIAS' was
  unhandled in the switch stmt.

2009-06-17 11:38  timoore

* simgear/structure/: exception.cxx, exception.hxx: change MAX_PATH
  to max_path to avoid Windows compilation problems

2009-06-16 13:02  timoore

* simgear/structure/: exception.cxx, exception.hxx: overhaul
  sg_throwable to behave like a proper exception

  Make sg_throwable inherit from std::exception.

  change exception objects to contain C strings: exception objects
  should not include objects, like std::string, whose copy
  constructor could throw.

2009-06-14 12:56  fredb

* projects/VC7.1/SimGear.vcproj, simgear/math/Makefile.am,
  simgear/math/beziercurve.hxx: Add a simple class to subdivide
  Bezier curves

2009-06-14 12:53  fredb

* simgear/scene/tgdb/GroundLightManager.cxx: Remove unused variable

2009-06-11 20:53  frohlich

* simgear/scene/model/: SGPagedLOD.cxx, modellib.hxx, modellib.cxx,
  SGReaderWriterXML.cxx: Finally get rid of that member in the
  SGModelData callback.  Move call of SGModelData::modelLoaded
  directly into the xml reader.

  Modified Files:	  simgear/scene/model/SGPagedLOD.cxx
  simgear/scene/model/modellib.hxx
  simgear/scene/model/modellib.cxx
  simgear/scene/model/SGReaderWriterXML.cxx

2009-06-11 20:32  frohlich

* simgear/scene/tgdb/TileEntry.cxx: Mark static transforms as
  static.

  Modified Files:	  simgear/scene/tgdb/TileEntry.cxx

2009-06-11 15:39  frohlich

* simgear/scene/model/: SGPagedLOD.cxx, SGReaderWriterXML.cxx,
  modellib.cxx, modellib.hxx: Revert a change from 2009/06/07.
  Should make the Nasal code for some xml models work again.

  Modified Files:	  simgear/scene/model/SGPagedLOD.cxx
  simgear/scene/model/SGReaderWriterXML.cxx
  simgear/scene/model/modellib.cxx
  simgear/scene/model/modellib.hxx

2009-06-11 10:42  frohlich

* simgear/scene/util/SGUpdateVisitor.hxx: Also handle PagedLOD
  nodes frame count in the update visitor.

  Modified Files:	  simgear/scene/util/SGUpdateVisitor.hxx

2009-06-11 09:15  frohlich

* simgear/scene/util/SGUpdateVisitor.hxx: Also test against bounds
  when updating the scene.

  Modified Files:	  simgear/scene/util/SGUpdateVisitor.hxx

2009-06-11 00:46  jmt

* simgear/route/waytest.cxx: Fix waypoint test, thanks Martin.

2009-06-10 14:42  jmt

* simgear/route/: route.cxx, route.hxx, waypoint.cxx, waypoint.hxx:
  Extend SGWaypoint with track and speed data, and compute tracks
  with the distance in SGRoute.

2009-06-09 22:51  fredb

* simgear/route/route.cxx: Win32 fix

2009-06-09 02:01  jmt

* simgear/route/: waypoint.cxx, waypoint.hxx: Fix bad interaction
  between CourseAndDistance overloads and use of implicit
  SGWaypoint construction from SGGeod.

2009-06-09 01:30  jmt

* simgear/route/: routetest.cxx, waytest.cxx: Update route/waypoint
  tests for revised API

2009-06-09 01:18  jmt

* simgear/route/: route.cxx, route.hxx, waypoint.cxx, waypoint.hxx:
  Change SGWaypoint to use SGGeod internally. Remove some unused
  code, to support cartesian waypoints and compute distance off a
  cartesian route.  Add a helper to access the total route
  distance.

  Should not cause any visible functionality change.

2009-06-07 13:58  frohlich

* simgear/scene/model/SGReaderWriterXML.cxx: More cleanup.

  Modified Files:	  simgear/scene/model/SGReaderWriterXML.cxx

2009-06-07 13:50  frohlich

* simgear/scene/model/ModelRegistry.cxx: Do not modify danymically
  generated textures.

  Modified Files:	  simgear/scene/model/ModelRegistry.cxx

2009-06-07 13:30  frohlich

* simgear/scene/model/: SGPagedLOD.cxx, modellib.cxx: Provide
  something more sensible for the properties root for the
  modelLoaded call. MAy be this needs to be revisited, but in any
  case better than constant zero.

  Modified Files:	  simgear/scene/model/SGPagedLOD.cxx
  simgear/scene/model/modellib.cxx

2009-06-07 13:27  frohlich

* simgear/scene/model/SGReaderWriterXML.cxx: Cleanup.

  Modified Files:	  simgear/scene/model/SGReaderWriterXML.cxx

2009-06-07 13:26  frohlich

* simgear/scene/model/: SGPagedLOD.cxx, SGReaderWriterXML.cxx,
  modellib.cxx, modellib.hxx: Doing the compilers job: constant
  propagation.	This variable is nowhere set except to zero.

  Modified Files:	  model/SGPagedLOD.cxx
  model/SGReaderWriterXML.cxx	     model/modellib.cxx
  model/modellib.hxx

2009-06-06 12:38  frohlich

* simgear/scene/model/SGReaderWriterXML.cxx: Slight cleanup.
  Remive last reference to plibs file utility library.

  Modified Files:	  simgear/scene/model/SGReaderWriterXML.cxx

2009-06-06 11:17  frohlich

* simgear/scene/tgdb/TileEntry.cxx: Remove unused header.

  Modified Files:	  simgear/scene/tgdb/TileEntry.cxx

2009-06-06 11:16  frohlich

* simgear/scene/material/mat.cxx: Use osgDB::FileUtils instead of
  plib file utils.

  Modified Files:	  simgear/scene/material/mat.cxx

2009-06-06 10:07  frohlich

* SimGear.dsp, projects/VC7.1/SimGear.vcproj,
  simgear/scene/model/Makefile.am,
  simgear/scene/model/placement.cxx,
  simgear/scene/model/placement.hxx,
  simgear/scene/model/placementtrans.cxx,
  simgear/scene/model/placementtrans.hxx,
  simgear/scene/tgdb/TileEntry.cxx,
  simgear/scene/tgdb/TileEntry.hxx: Replace SGPlacementTrans usage
  with osg::PositionAttitueTransform.  Remove SGPlacementTrans.
  Update build system.

  Modified Files:	  SimGear.dsp projects/VC7.1/SimGear.vcproj
  projects/VC8/SimGear.vcproj simgear/scene/model/Makefile.am
      simgear/scene/model/placement.cxx
  simgear/scene/model/placement.hxx
  simgear/scene/tgdb/TileEntry.cxx
  simgear/scene/tgdb/TileEntry.hxx Removed Files:
  simgear/scene/model/placementtrans.cxx
  simgear/scene/model/placementtrans.hxx

2009-06-06 09:24  frohlich

* simgear/scene/: util/SGUpdateVisitor.hxx, tgdb/TileEntry.cxx,
  model/placementtrans.cxx: Use osg's builtin mechanisms to
  traverse only in range nodes with the update visitor.

  Modified Files:	  simgear/scene/util/SGUpdateVisitor.hxx
  simgear/scene/tgdb/TileEntry.cxx
  simgear/scene/model/placementtrans.cxx

2009-06-03 21:30  frohlich

* simgear/scene/model/SGReaderWriterXML.cxx: Invent a property root
  if not given in the options struct.

  Modified Files:	  simgear/scene/model/SGReaderWriterXML.cxx

2009-05-25 13:12  ehofman

* simgear/props/props_test.cxx: test for addChild also

2009-05-25 13:12  ehofman

* simgear/props/props.cxx: find the last index instead of the last
  pos for addChild

2009-05-25 06:59  frohlich

* simgear/scene/tgdb/TileEntry.cxx: Avoid empty search path
  extensions for the case that only the bucket index number is
  given.

  Modified Files:	  scene/tgdb/TileEntry.cxx

2009-05-24 08:38  frohlich

* simgear/scene/tgdb/: ReaderWriterSTG.cxx, TileEntry.cxx,
  TileEntry.hxx: When loading stg files honour the original path if
  given.

  Modified Files:	  simgear/scene/tgdb/ReaderWriterSTG.cxx
  simgear/scene/tgdb/TileEntry.cxx
  simgear/scene/tgdb/TileEntry.hxx

2009-05-23 14:00  frohlich

* simgear/scene/tgdb/ReaderWriterSTG.cxx: Enable stg loading by
  filename.

  Modified Files:	  simgear/scene/tgdb/ReaderWriterSTG.cxx

2009-05-23 09:32  mfranz

* simgear/scene/model/modellib.cxx: segfault--

2009-05-22 20:20  frohlich

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  material/matlib.cxx, material/matlib.hxx,
  tgdb/ReaderWriterSTG.cxx, tgdb/SGReaderWriterBTG.cxx,
  tgdb/SGReaderWriterBTGOptions.hxx, tgdb/TileEntry.cxx,
  tgdb/TileEntry.hxx, tgdb/obj.cxx: Cleanup.  Additional null
  pointer checks.  Simplify redundant interface arguments.

  Modified Files:	  simgear/scene/material/mat.cxx
  simgear/scene/material/mat.hxx
  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx
  simgear/scene/tgdb/ReaderWriterSTG.cxx
  simgear/scene/tgdb/SGReaderWriterBTG.cxx
  simgear/scene/tgdb/SGReaderWriterBTGOptions.hxx
  simgear/scene/tgdb/TileEntry.cxx
  simgear/scene/tgdb/TileEntry.hxx simgear/scene/tgdb/obj.cxx

2009-05-22 16:53  frohlich

* simgear/scene/model/SGPagedLOD.cxx: Give the models properties as
  an argument to the init callback.

  Modified Files:	  SGPagedLOD.cxx

2009-05-19 23:29  mfranz

* simgear/scene/tgdb/Makefile.am: remove duplicated header entry

2009-05-19 07:30  frohlich

* simgear/scene/tgdb/Makefile.am: Also note the btg reader header
  in the build system.

  Modified Files:	  scene/tgdb/Makefile.am

2009-05-19 07:30  frohlich

* simgear/scene/tgdb/: SGReaderWriterBTG.cxx,
  SGReaderWriterBTG.hxx: Restore the special capability of the btg
  reader to read compressed files.

  Modified Files:	  scene/tgdb/SGReaderWriterBTG.cxx
  scene/tgdb/SGReaderWriterBTG.hxx

2009-05-16 20:12  frohlich

* simgear/scene/tgdb/SGReaderWriterBTG.hxx: Implement current
  osgDB::ReaderWriters supportsExtension interface instead of the
  previous one.

  Modified Files:	  model/SGReaderWriterXML.cxx
  model/SGReaderWriterXML.hxx	      tgdb/ReaderWriterSTG.cxx
  tgdb/ReaderWriterSTG.hxx	 tgdb/SGReaderWriterBTG.cxx
  tgdb/SGReaderWriterBTG.hxx

2009-05-16 20:08  frohlich

* simgear/scene/tgdb/: ReaderWriterSTG.hxx, SGReaderWriterBTG.cxx:
  Implement current osgDB::ReaderWriters supportsExtension
  interface instead of the previous one.

  Modified Files:	  model/SGReaderWriterXML.cxx
  model/SGReaderWriterXML.hxx	      tgdb/ReaderWriterSTG.cxx
  tgdb/ReaderWriterSTG.hxx	 tgdb/SGReaderWriterBTG.cxx
  tgdb/SGReaderWriterBTG.hxx

2009-05-16 20:05  frohlich

* simgear/scene/: model/SGReaderWriterXML.cxx,
  model/SGReaderWriterXML.hxx, tgdb/ReaderWriterSTG.cxx: Implement
  current osgDB::ReaderWriters supportsExtension interface instead
  of the previous one.

  Modified Files:	  model/SGReaderWriterXML.cxx
  model/SGReaderWriterXML.hxx	      tgdb/ReaderWriterSTG.cxx
  tgdb/ReaderWriterSTG.hxx	 tgdb/SGReaderWriterBTG.cxx
  tgdb/SGReaderWriterBTG.hxx

2009-05-16 19:46  frohlich

* simgear/scene/model/: SGPagedLOD.cxx, SGReaderWriterXML.cxx:
  Attach the ModelData to the options instead of the userdata
  field.

  Modified Files:	  simgear/scene/model/SGPagedLOD.cxx
  simgear/scene/model/SGReaderWriterXML.cxx

2009-05-16 10:13  frohlich

* simgear/: scene/model/SGPagedLOD.cxx, scene/model/SGPagedLOD.hxx,
  scene/sky/cloud.cxx, scene/sky/moon.cxx,
  structure/OSGVersion.hxx: Make SimGear compile with osg trunk

  Modified Files:	  simgear/scene/model/SGPagedLOD.cxx
  simgear/scene/model/SGPagedLOD.hxx
  simgear/scene/sky/cloud.cxx simgear/scene/sky/moon.cxx
  simgear/structure/OSGVersion.hxx

2009-05-12 17:56  mfranz

* simgear/: debug/logtest.cxx, math/interpolater.hxx,
  misc/tabbed_values.cxx, threads/SGThread.cxx, xml/xmldef.h: - fix
  one broken #include path (in a not usually compiled test app) -
  turn four #include paths from the "foo" form to <foo>

  The quotes form is normally only used for headers with path
  relative to the including file's path, though the standard
  doesn't strictly mandate this. This is consistent with the rest
  of sg, it makes the code's intent clearer and helps to find
  headers. (And it's a few milliseconds faster, too.)

2009-05-09 13:34  ehofman

* simgear/props/: props.cxx, props.hxx:
    * Add a function to create a node after the laste node with the
  same name
      (this frees the xml property loader from keeping track of the
  number of
      nodes with the same name that have already been added to the
  property
      tree).
    * make some small code cleanups at the core of the property
  tree.

2009-05-06 18:17  mfranz

* simgear/io/tcp_server.cxx: tcpserver: typo

2009-05-06 08:32  mfranz

* simgear/: io/tcp_server.cxx, screen/screen-dump.cxx,
  timing/sg_time.cxx: fix leaks

2009-04-25 11:57  ehofman

* simgear/sound/xmlsound.cxx: add a bit of comment

2009-04-18 20:47  fredb

* simgear/scene/tgdb/TreeBin.cxx: Stuart Buchanan : This patch
  changes the shader so the diffuse light element is applied based
  on the co-linearity of the light vector and the viewing vector. I
  think this makes sense, as the tree textures don't represent a
  surface themselves.

2009-04-12 13:42  ehofman

* simgear/scene/sky/dome.cxx: Revert most of the previous patch, it
  didn't improve the dome since it wat adjusting the wrong bads

2009-04-12 11:46  frohlich

* simgear/scene/model/modellib.cxx: Put some annotations into the
  loaded models names.

  Modified Files:	  simgear/scene/model/modellib.cxx

2009-04-12 10:05  frohlich

* simgear/scene/sky/dome.cxx: Remove a newline printf, probably
  left over from development.

  Modified Files:	  dome.cxx

2009-04-11 14:27  ehofman

* simgear/scene/sky/: dome.cxx, dome.hxx, sky.cxx, sky.hxx: Let the
  fog color transition into the sky dome to give a more natural
  effect

2009-04-06 21:24  ehofman

* simgear/scene/sky/oursun.cxx: Fix a NaN at higher altitudes:
  sin_beta could become greater than 1.0 which is hard to
  understand for asin.

2009-04-02 07:38  frohlich

* simgear/bucket/newbucket.hxx: Remove unused variables.

  Modified Files:	  simgear/bucket/newbucket.hxx

2009-03-27 23:38  torsten

* simgear/bucket/newbucket.cxx: fix wrong bucket calculation for
  western hemisphere, close to poles, full-degree-longitude
  calculations.  set_bucket computed false left border, if the span
  was greater than 1, longitudes were less than zero and longitudes
  were within SG_EPSILON on a full degree. Example: -179.0, +87.5
  returned -176, 87 but should return -180, 87.  Discovered by
  Brian Schack

2009-03-24 10:09  frohlich

* simgear/scene/model/placement.cxx: Makes more sense to process
  the orientation in this order.

  Modified Files:	  simgear/scene/model/placement.cxx

2009-03-24 09:12  frohlich

* simgear/scene/model/: placement.cxx, placementtrans.cxx,
  placementtrans.hxx: Clean up placementtrans a bit.

  Modified Files:	  simgear/scene/model/placement.cxx
  simgear/scene/model/placementtrans.cxx
  simgear/scene/model/placementtrans.hxx

2009-03-24 09:11  frohlich

* simgear/io/lowlevel.cxx: Zap aliasing compiler warnings.

  Modified Files:	  simgear/io/lowlevel.cxx

2009-03-24 09:10  frohlich

* simgear/scene/model/SGScaleTransform.cxx: Remove second,
  identical to the first one license header.

  Modified Files:	  simgear/scene/model/SGScaleTransform.cxx

2009-03-24 09:09  frohlich

* simgear/scene/tgdb/TileEntry.cxx: Remove unused header.

  Modified Files:	  simgear/scene/tgdb/TileEntry.cxx

2009-03-24 09:09  frohlich

* simgear/debug/logstream.hxx: Make it compile with gcc-4.4.

  Modified Files:	  simgear/debug/logstream.hxx

2009-03-24 09:03  frohlich

* simgear/scene/model/SGTranslateTransform.cxx: Make use of
  optimized matrix multiplication routines in osg.

  Modified Files:
  simgear/scene/model/SGTranslateTransform.cxx

2009-03-19 08:58  frohlich

* simgear/scene/model/ModelRegistry.cxx: No longer change the
  material properties for ac models on the fly.

  Modified Files:	  simgear/scene/model/ModelRegistry.cxx

2009-03-17 13:25  frohlich

* simgear/scene/sky/: moon.cxx, moon.hxx, oursun.cxx, oursun.hxx,
  sky.cxx, sky.hxx, stars.cxx, stars.hxx: Cleanup. Use already
  structured data instead of copying stuff.

2009-03-17 13:23  frohlich

* simgear/ephemeris/ephemeris.hxx: Add const accessors.

  Modified Files:	  simgear/ephemeris/ephemeris.hxx

2009-03-15 16:06  frohlich

* simgear/scene/tgdb/SGReaderWriterBTG.cxx: Make sure the
  boundingvolumes for the btg files are as high as possible in the
  scenegraph.

  Modified Files:	  simgear/scene/tgdb/SGReaderWriterBTG.cxx

2009-03-15 13:56  frohlich

* simgear/scene/bvh/BVHSubTreeCollector.cxx: Use triangle sphere
  intersection directly.

  Modified Files:	  simgear/scene/bvh/BVHSubTreeCollector.cxx

2009-03-15 13:50  frohlich

* simgear/math/SGIntersect.hxx: Make triangle sphere intersection
  tests usable with mixes types.

  Modified Files:	  simgear/math/SGIntersect.hxx

2009-03-15 12:03  frohlich

* simgear/scene/bvh/: BVHMotionTransform.cxx,
  BVHMotionTransform.hxx, BVHSubTreeCollector.cxx: Preparations for
  improoved timing behaviour.

  Modified Files:	  simgear/scene/bvh/BVHMotionTransform.cxx
  simgear/scene/bvh/BVHMotionTransform.hxx
  simgear/scene/bvh/BVHSubTreeCollector.cxx

2009-03-14 13:26  frohlich

* simgear/scene/: model/placement.cxx, model/placement.hxx,
  util/SGSceneUserData.hxx: Add a reference time to the velocity
  stuff.

  Modified Files:	  simgear/scene/model/ModelRegistry.cxx
  simgear/scene/model/placement.cxx
  simgear/scene/model/placement.hxx
  simgear/scene/util/SGSceneUserData.hxx

2009-03-14 12:08  frohlich

* simgear/scene/tgdb/obj.cxx: The chunked stuff is no longer needed
  for the ground tiles.

  ... I (Mathias Froehlich), believe that it is more performant
  (for the GPU) to build the biggest indexed sets possible at the
  expense of tight culling.  I (Mathias Froehlich) also know that
  the CPU is more performant on smaller chunks building the old
  flat groundcache on such tiles. :) So this code *was* to balance
  these two contrary requirements to some degree.

  Now we have optimized datastructures for drawing and differently
  optimized datastructures for intersection tests.

  Modified Files:	  simgear/scene/tgdb/obj.cxx

2009-03-14 12:00  frohlich

* simgear/scene/tgdb/SGOceanTile.cxx: Also align the ocean tiles
  with some horizontal axis.  This way the bounding boxes for this
  geometry is slightly smaller.  Helps ground intersection stuff to
  avoid the ocean triangles earlier.

  Modified Files:	  simgear/scene/tgdb/SGOceanTile.cxx

2009-03-14 10:17  frohlich

* simgear/scene/model/: BoundingVolumeBuildVisitor.hxx,
  ModelRegistry.cxx, ModelRegistry.hxx: Improove bounding volume
  building in the scenery loading process.  Refactor common code in
  the BoundingVolumeBuildVisitor.hxx.

  Modified Files:
  simgear/scene/model/BoundingVolumeBuildVisitor.hxx
  simgear/scene/model/ModelRegistry.cxx
  simgear/scene/model/ModelRegistry.hxx

2009-03-14 10:16  frohlich

* simgear/scene/bvh/BVHDebugCollectVisitor.hxx: Make the debug
  geometry stuff work with a time argument.

  Modified Files:
  simgear/scene/bvh/BVHDebugCollectVisitor.hxx

2009-03-13 08:33  fredb

* simgear/timing/timestamp.cxx: Compile under windows

2009-03-13 06:45  frohlich

* simgear/scene/model/ModelRegistry.cxx: Make sure each carrier
  gets its own valocity.

  Modified Files:	  simgear/scene/model/ModelRegistry.cxx

2009-03-12 21:07  mfranz

* simgear/timing/timestamp.hxx: warning--

2009-03-12 19:34  frohlich

* simgear/: structure/subsystem_mgr.cxx,
  structure/subsystem_mgr.hxx, timing/testtimestamp.cxx,
  timing/timestamp.cxx, timing/timestamp.hxx: Make use of posix
  clocks if available.	Have a more general timestamp
  implementation.  Very useful for higher accuracy timings.

  Modified Files:	  simgear/structure/subsystem_mgr.cxx
  simgear/structure/subsystem_mgr.hxx
  simgear/timing/testtimestamp.cxx simgear/timing/timestamp.cxx
  simgear/timing/timestamp.hxx

2009-03-11 06:34  frohlich

* simgear/sound/: xmlsound.hxx, xmlsound.cxx: Use a reference
  counted pointer for the condition.

  Modified Files:	  simgear/sound/xmlsound.hxx
  simgear/sound/xmlsound.cxx

2009-03-10 20:23  frohlich

* simgear/scene/material/: matmodel.hxx, matmodel.cxx: Remove
  unused methods.

  Modified Files:	  material/matmodel.hxx
  material/matmodel.cxx

2009-03-10 20:22  frohlich

* simgear/scene/model/SGReaderWriterXML.cxx: Make local functions
  static.

  Modified Files:	  model/SGReaderWriterXML.cxx

2009-03-10 20:20  frohlich

* simgear/scene/tgdb/: userdata.cxx, userdata.hxx: Remove unused
  functions/includes ...

  Modified Files:	  simgear/scene/tgdb/userdata.hxx
  simgear/scene/tgdb/userdata.cxx

2009-03-07 22:47  frohlich

* projects/VC7.1/SimGear.vcproj, simgear/scene/model/Makefile.am,
  simgear/scene/model/location.cxx,
  simgear/scene/model/location.hxx,
  simgear/scene/model/placement.cxx,
  simgear/scene/model/placement.hxx: Zap SGLocation.

  Modified Files:	  projects/VC7.1/SimGear.vcproj
  projects/VC8/SimGear.vcproj	    simgear/scene/model/Makefile.am
    simgear/scene/model/placement.cxx
  simgear/scene/model/placement.hxx Removed Files:
  simgear/scene/model/location.cxx
  simgear/scene/model/location.hxx

2009-03-07 22:45  frohlich

* simgear/scene/model/BoundingVolumeBuildVisitor.hxx: Ignore
  winding order since it is ignored later anyway.  Only emit fully
  defined primitives.

  Modified Files:
  simgear/scene/model/BoundingVolumeBuildVisitor.hxx

2009-03-07 12:11  frohlich

* simgear/scene/model/: placement.cxx, placement.hxx: Use SGGeod in
  the model placement.

  Modified Files:	  simgear/scene/model/placement.cxx
  simgear/scene/model/placement.hxx

2009-03-07 12:10  frohlich

* simgear/math/SGGeod.hxx: Add convenience function to keep
  position with just an other elevation.

  Modified Files:	  simgear/math/SGGeod.hxx

2009-03-05 07:06  frohlich

* simgear/scene/: bvh/BVHLineSegmentVisitor.cxx,
  bvh/BVHLineSegmentVisitor.hxx, bvh/BVHMotionTransform.cxx,
  bvh/BVHMotionTransform.hxx, bvh/BVHNearestPointVisitor.hxx,
  bvh/BVHNode.cxx, bvh/BVHNode.hxx, util/SGSceneUserData.hxx: First
  step for something doing static friction stuff.

  Add an id field to identify BVHMotionTransforms.  Provide a
  factory for ids.  Use that to identify velocity data.  Track the
  lowermost id in the visitors.

  Modified Files:
  simgear/scene/bvh/BVHLineSegmentVisitor.cxx
  simgear/scene/bvh/BVHLineSegmentVisitor.hxx
  simgear/scene/bvh/BVHMotionTransform.cxx
  simgear/scene/bvh/BVHMotionTransform.hxx
  simgear/scene/bvh/BVHNearestPointVisitor.hxx
  simgear/scene/bvh/BVHNode.cxx simgear/scene/bvh/BVHNode.hxx
  simgear/scene/util/SGSceneUserData.cxx
  simgear/scene/util/SGSceneUserData.hxx

2009-03-03 09:25  frohlich

* simgear/scene/bvh/BVHStaticGeometryBuilder.hxx: Avoid duplicate
  triangles.

  Modified Files:
  simgear/scene/bvh/BVHStaticGeometryBuilder.hxx

2009-03-02 19:02  frohlich

* simgear/scene/bvh/: BVHBoundingBoxVisitor.hxx,
  BVHDebugCollectVisitor.hxx, BVHLineSegmentVisitor.cxx,
  BVHLineSegmentVisitor.hxx, BVHNearestPointVisitor.hxx,
  BVHStaticLeaf.cxx, BVHStaticLeaf.hxx, BVHSubTreeCollector.cxx,
  BVHSubTreeCollector.hxx, BVHVisitor.hxx, Makefile.am,
  bvhtest.cxx: Remove the StaticLeaf visitor slot.  Add a nearest
  point visitor.

  Modified Files:	  BVHBoundingBoxVisitor.hxx
  BVHDebugCollectVisitor.hxx	BVHLineSegmentVisitor.cxx
  BVHLineSegmentVisitor.hxx	BVHStaticGeometryBuilder.hxx
  BVHStaticLeaf.cxx  BVHStaticLeaf.hxx BVHSubTreeCollector.cxx
    BVHSubTreeCollector.hxx BVHVisitor.hxx Makefile.am bvhtest.cxx
  Added Files:	   BVHNearestPointVisitor.hxx

2009-03-01 22:31  frohlich

* simgear/scene/model/ModelRegistry.cxx: Crude hack to work around
  my today introduced collision tree sharing problem.

  Modified Files:	  simgear/scene/model/ModelRegistry.cxx

2009-03-01 22:25  frohlich

* simgear/scene/bvh/BVHStaticGeometryBuilder.hxx: Don't waste space
  with too huge stl containers.

  Modified Files:
  simgear/scene/bvh/BVHStaticGeometryBuilder.hxx

2009-03-01 22:25  frohlich

* simgear/scene/bvh/BVHStaticData.hxx: Don't waste space with too
  huge stl containers.

  Modified Files:	  simgear/scene/bvh/BVHStaticData.hxx

2009-03-01 16:50  fredb

* simgear/scene/model/ModelRegistry.cxx: Win32 fix

2009-03-01 16:40  frohlich

* simgear/scene/model/: ModelRegistry.hxx, ModelRegistry.cxx,
  BoundingVolumeBuildVisitor.hxx: Build boundingvolumes in the
  model loading phase.

  Modified Files:	  ModelRegistry.hxx ModelRegistry.cxx Added
  Files:	BoundingVolumeBuildVisitor.hxx

2009-03-01 13:50  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects - Add the
  BVH library

2009-03-01 13:49  fredb

* simgear/math/SGMatrix.hxx: SGMatrix<T>::preMultTranslate should
  return self ( as postMultTranslate do )

2009-03-01 13:48  fredb

* simgear/scene/bvh/BVHLineSegmentVisitor.hxx:
  BVHLineSegmentVisitor::setLineSegmentEnd doesn't return anything

2009-03-01 13:40  frohlich

* simgear/scene/model/: placement.cxx, placement.hxx: Implement
  setters for velocity notes in the model placement code.

  Modified Files:	  simgear/scene/model/placement.cxx
  simgear/scene/model/placement.hxx

2009-03-01 13:37  frohlich

* simgear/scene/util/SGSceneUserData.hxx: More data to attach to
  the user data field.

  Modified Files:	  simgear/scene/util/SGSceneUserData.hxx

2009-03-01 13:36  frohlich

* configure.ac, simgear/scene/Makefile.am: Include the bounding
  volume tree on the build system.

  Modified Files:	  configure.ac simgear/scene/Makefile.am

2009-03-01 13:35  frohlich

* simgear/scene/bvh/: .cvsignore, BVHBoundingBoxVisitor.hxx,
  BVHDebugCollectVisitor.hxx, BVHGroup.cxx, BVHGroup.hxx,
  BVHLineGeometry.cxx, BVHLineGeometry.hxx,
  BVHLineSegmentVisitor.cxx, BVHLineSegmentVisitor.hxx,
  BVHMotionTransform.cxx, BVHMotionTransform.hxx, BVHNode.cxx,
  BVHNode.hxx, BVHStaticBinary.cxx, BVHStaticBinary.hxx,
  BVHStaticData.hxx, BVHStaticGeometry.cxx, BVHStaticGeometry.hxx,
  BVHStaticGeometryBuilder.hxx, BVHStaticLeaf.cxx,
  BVHStaticLeaf.hxx, BVHStaticNode.cxx, BVHStaticNode.hxx,
  BVHStaticTriangle.cxx, BVHStaticTriangle.hxx,
  BVHSubTreeCollector.cxx, BVHSubTreeCollector.hxx,
  BVHTransform.cxx, BVHTransform.hxx, BVHVisitor.hxx, Makefile.am,
  bvhtest.cxx: Initial commit of the bounding volume tree
  implementation.  The aim is to prove a better collion model in
  plenty ways.

  Added Files:	  .cvsignore BVHBoundingBoxVisitor.hxx
  BVHDebugCollectVisitor.hxx BVHGroup.cxx BVHGroup.hxx
  BVHLineGeometry.cxx BVHLineGeometry.hxx
  BVHLineSegmentVisitor.cxx BVHLineSegmentVisitor.hxx
  BVHMotionTransform.cxx BVHMotionTransform.hxx BVHNode.cxx
  BVHNode.hxx BVHStaticBinary.cxx BVHStaticBinary.hxx
  BVHStaticData.hxx BVHStaticGeometry.cxx BVHStaticGeometry.hxx
  BVHStaticGeometryBuilder.hxx BVHStaticLeaf.cxx  BVHStaticLeaf.hxx
  BVHStaticNode.cxx BVHStaticNode.hxx	BVHStaticTriangle.cxx
  BVHStaticTriangle.hxx     BVHSubTreeCollector.cxx
  BVHSubTreeCollector.hxx	  BVHTransform.cxx BVHTransform.hxx
  BVHVisitor.hxx Makefile.am	bvhtest.cxx

2009-03-01 13:22  frohlich

* simgear/math/: SGGeometryTest.cxx, SGIntersect.hxx,
  SGLineSegment.hxx, SGTriangle.hxx: Additions for the
  boundingvolumes Modified Files:      SGGeometryTest.cxx
  SGIntersect.hxx SGLineSegment.hxx    SGTriangle.hxx

2009-02-27 20:59  frohlich

* simgear/scene/tgdb/obj.cxx: Rotate the scenery tiles so that the
  horizont is axis aligned.  This might help culling and much more
  the upcomming collision tree.

  Modified Files:	  simgear/scene/tgdb/obj.cxx

2009-02-27 20:30  frohlich

* simgear/scene/material/: matlib.cxx, matlib.hxx: Make material
  get helper function static.

  Modified Files:	  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx

2009-02-27 19:52  frohlich

* simgear/math/: SGGeometryTest.cxx, SGTriangle.hxx: Initialize
  random number for the tests.	Define the triangles center by the
  weighted sum of the vertices.

  Modified Files:	  SGGeometryTest.cxx SGTriangle.hxx

2009-02-27 07:05  frohlich

* simgear/math/SGTriangle.hxx: Use template arguments instead of
  hard coded double.

  Modified Files:	  simgear/math/SGTriangle.hxx

2009-02-27 06:18  frohlich

* simgear/math/SGMathTest.cxx: Initialize uninitialized variables.
  Adapt the precision bounds to what matches the expectations of
  IEEE math.

  Modified Files:	  SGMathTest.cxx

2009-02-27 06:16  frohlich

* simgear/math/SGQuat.hxx: Fix problem in unit quaternion return.

  Modified Files:	  SGQuat.hxx

2009-02-27 06:16  frohlich

* simgear/math/Makefile.am: Make make check link again.

  Modified Files:	  simgear/math/Makefile.am

2009-02-22 10:15  frohlich

* simgear/scene/model/: placement.hxx, placementtrans.cxx,
  placementtrans.hxx: The scenery center is history for a long time
  now.	Remove that from the transforms.

  Modified Files:	  simgear/scene/model/placement.hxx
  simgear/scene/model/placementtrans.cxx
  simgear/scene/model/placementtrans.hxx

2009-02-22 09:13  frohlich

* simgear/math/: SGBox.hxx, SGIntersect.hxx, SGLineSegment.hxx,
  SGMatrix.hxx, SGPlane.hxx, SGRay.hxx, SGSphere.hxx, SGVec2.hxx,
  SGVec3.hxx, SGVec4.hxx: Small cleanups, bug fixes and
  improovements to the geometry/math stuff.

  Modified Files:	  simgear/math/SGBox.hxx
  simgear/math/SGIntersect.hxx	   simgear/math/SGLineSegment.hxx
  simgear/math/SGMatrix.hxx	   simgear/math/SGPlane.hxx
  simgear/math/SGRay.hxx	 simgear/math/SGSphere.hxx
  simgear/math/SGVec2.hxx	simgear/math/SGVec3.hxx
  simgear/math/SGVec4.hxx

2009-02-21 10:11  durk

* simgear/scene/model/animation.cxx: Torsten Dreyer: the rotate
  animation has two ways to define a axis 1. by using center/x-m
  and axis/[xyz] 2. by using axis/[xyz]1-m and axis/[xyz]2-m

  The translate animation only supports method 1. and here comes a
  patch that enables method 2 for this animations, too.

2009-02-19 21:41  fredb

* simgear/nasal/iolib.c: Fix to compile with MSC

2009-02-15 17:54  mfranz

* simgear/nasal/: iolib.c, iolib.h: "io.flush() implementation from
  Melchior.  Also add a final filetype element to the io.stat()
  array, which has been sitting around on my box for a while and
  doesn't seem to have broken anything."  -- Andy

  (forward port from nasal/cvs)

2009-02-13 10:09  timoore

* projects/VC7.1/SimGear.vcproj, simgear/scene/util/Makefile.am,
  simgear/scene/util/PrimitiveUtils.cxx,
  simgear/scene/util/PrimitiveUtils.hxx: New getPrimitive function.

  This searches for the primitive found in an OSG intersection
  query.

2009-02-07 08:47  timoore

* simgear/structure/: subsystem_mgr.cxx, subsystem_mgr.hxx: Make
  SGSubsystemGroup::Member copy constructor private.

  The copy constructor wouldn't make any sense and would confuse
  ownership of the subsystem pointer. Problem noted by John Denker.

2009-02-06 22:08  timoore

* simgear/scene/: sky/newcloud.cxx, util/StateAttributeFactory.cxx,
  util/StateAttributeFactory.hxx: Turn off z buffer writes for
  clouds.

  This is standard practice for semi-transparent objects and should
  cut down on the flickering and other sorting artifacts.

2009-02-02 06:34  durk

* configure.ac: Synchronize the version number with our mainantance
  release.

2009-01-30 11:22  timoore

* simgear/scene/tgdb/TreeBin.cxx: Build trees under a transform
  note that is rotated to Z-up.

  This allows the tree geometry to be shared across the entire
  scene because it doesn't need to be rotated for each scenery
  tile.

2009-01-30 11:22  timoore

* simgear/scene/tgdb/: ShaderGeometry.cxx, ShaderGeometry.hxx:
  Don't expand all the trees into display lists.

  This can chew up large amounts of memory for questionable gains.
  We do let the tree model geometry be in a display list if OSG
  chooses to put it there.

  Various renaming and cleanup. Save some memory by reverting
  ShaderGeometry's base class back to osg::Drawable.

2009-01-28 08:09  timoore

* simgear/scene/tgdb/: ShaderGeometry.cxx, ShaderGeometry.hxx: Use
  osg::Geometry code to draw trees ourselves.

  This reverts back in the direction of Yon's original patch: the
  model is drawn within ShaderGeometry::drawImplementation. This
  saves a lot of memory over the previous approach of creating a
  PrimitiveSet for each model.

2009-01-27 23:43  durk

* simgear/timing/: sg_time.cxx, sg_time.hxx:  For the first time
  (no pun intended) in almost ten years time (again no
    pun intended) that I'm touching the time library.

    Brian Schack reported that the traffic scheduler messes up the
  timestamps
    of the atlas network output. As it turns out, the c library
  functions
    asctime, and gmtime use a static copy of the tm struct to do the
  internal
    formatting. Our linux port of the SGTime class, incidentally,
  also stored
    it's master time stamp in this very same struct. Thus,
  formatting an
    arbitrary time value, would have the unwanted side effect of
  time travel.
    Usually, this would go unnoticed, because the actual time
  parameters would
    be updated before any damage could be done. But unwanted side
  effects, as
    in Brian's example could occur.

    On the MSVC port this appears to not have been a problem. Since
  that port
    used a copy of the tm struct to store it's master time stamps.
  Since the
    MSVC code also compiles cleanly on linux, it seems to be the way
  to go to
    use that approach. In addition, it also removes some conditional
  compile
    directives.

    I've only run a short test, but didn't see any undesirable side
  effects.

2009-01-26 19:01  mfranz

* simgear/structure/SGBinding.cxx: Csaba HALASZ: set _arg even if
  there's no <command> (crash fix)

2009-01-25 12:53  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects - Remove
  pthreads dependency

2009-01-23 19:38  timoore

* simgear/scene/model/ModelRegistry.cxx: Optimize empty groups from
  .ac models

  The OSG optimizer seems to have changed since this work was
  originally done.

2009-01-23 19:36  timoore

* simgear/scene/util/QuadTreeBuilder.hxx: QuadTreeBuilder: create
  leaves on demand

  This avoids lots of empty leaves.

2009-01-23 19:35  timoore

* simgear/scene/tgdb/: ShaderGeometry.cxx, ShaderGeometry.hxx,
  TreeBin.cxx: Rewrite ShaderGeometry to use display lists and OSG
  primitives.

  Based on a patch from Yon Uriarte.

  Eliminate _trees list from ShaderGeometry

  Use the position and scale values that are already available.

2009-01-19 00:10  timoore

* simgear/scene/util/QuadTreeBuilder.hxx: Protect against division
  by zero in QuadTreeBuilder

  This could only happen when there's one leaf in the tree, or all
  the objects happen to have the same position. Noticed by Csaba
  Halaz

2009-01-16 00:48  timoore

* simgear/scene/sky/: CloudShaderGeometry.cxx,
  CloudShaderGeometry.hxx, cloudfield.cxx: Sort cloud sprites using
  std::sort, based on projected Z.

  A "cleanup" of cloud sorting.

2009-01-15 15:46  jmt

* simgear/: props/Makefile.am, sound/Makefile.am: Fix linkage of
  test programs with OpenThreads.

2009-01-15 15:34  jmt

* simgear/: threads/SGQueue.hxx, scene/util/SGSceneFeatures.cxx,
  scene/util/SGSceneFeatures.hxx: Commit Benoit Laniel's patch
  which converts more SimGear pieces to use OpenThreads primitives
  directly.

2009-01-15 15:31  jmt

* simgear/structure/: SGReferenced.hxx, commands.cxx, commands.hxx:
  Since we're now sure 1.9.1 will not be released from trunk,
  here's Yon Uriarte's patch to convert SGReferenced over to
  OpenThread's atomic int.

2009-01-14 22:16  timoore

* simgear/scene/model/ModelRegistry.cxx: Remove OptionsPusher and
  manipulation of osgDB dataFilePathList.

  The bug it worked around has been fixed in the OSG ac3d loader,
  and changing the path list has unpleasant multithreading
  implications.

2009-01-14 12:12  timoore

* simgear/props/props.cxx: SGPropertyNode must increment /
  decrement the reference counter in an aliased node.

  From Csaba Halász.

2009-01-13 22:42  timoore

* simgear/scene/model/SGMaterialAnimation.cxx: SGMaterialAnimation:
  Don't install an update callback if values are static

2009-01-13 22:41  timoore

* simgear/scene/model/SGMaterialAnimation.cxx: SGMaterialAnimation:
  factor out update of osg::Material

2009-01-13 08:49  fredb

* simgear/scene/model/SGMaterialAnimation.cxx: Csaba/Jester : fix
  the material animation and display night textures

2009-01-10 19:53  fredb

* projects/VC7.1/.cvsignore: fix end of file

2009-01-10 17:12  fredb

* projects/VC7.1/.cvsignore: ignore generated files

2008-12-27 09:16  timoore

* simgear/math/SGGeodesy.cxx: Fix include path

2008-12-26 13:27  jmt

* simgear/route/Makefile.am: Fix test-program linkage now sgmath
  depends on sgstructure.

2008-12-26 13:08  jmt

* simgear/math/: SGGeodesy.cxx, SGGeodesy.hxx: Add some syntactic
  helpers to allow distance/course to be queried between two
  geodetic points. This still entails a full _geo_inverse_wgs_84
  call, but makes call sites neater.

2008-12-26 12:42  jmt

* simgear/scene/model/ModelRegistry.cxx: Fix a potential crash when
  OSG is misconfigured, and an appropriate image loading plugin
  cannot be found.

2008-12-22 19:01  mfranz

* simgear/props/props_io.cxx: compilation fix: <cstring> for
  strcmp()

2008-12-21 16:01  mfranz

* simgear/scene/model/SGMaterialAnimation.cxx: - shininess is in
  the rage 0..128 - restore fg/plib compatibility (there is/was no
  <shininess> group)   (we may want to change that in the (near?)
  future, though)

2008-12-20 10:10  durk

* README.OSG, README.OpenAL: Documentation update regarding OpenAL
  and OSG.

2008-12-19 21:39  curt

* Doxyfile, NEWS, configure.ac: Attempt to sort out the version
  number mess in preparation for a 1.9.0 release.

2008-12-19 08:39  fredb

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx: Stuart
  Buchanan :

  Attached is yet another 3D clouds patch, to fix the following: 1)
  The 3D clouds were not modified by the cloud coverage, due to
  some problems with osg::Switch 2) METAR changes to cloud coverage
  were not obeyed.  3) Making changes via the Cloud dialog had no
  effect unless 3D clouds were toggled.  4) Cloud cover was too
  sparse.  5) 3D Stratus clouds caused performance issues on some
  hardware (fixed by removing 3D stratus from cloudlayers.xml - it
  will now be a 2D layer).

2008-12-18 23:51  mfranz

* simgear/route/route.hxx: James TURNER: make the index of the
  current waypoint available

2008-12-14 19:47  fredb

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx, newcloud.cxx:
  Stuart : the 3D clouds now respect changes to the environment
  caused by updated METAR etc.

  It also increases the AlphaFunc as suggested by Tim.

2008-12-12 21:02  fredb

* simgear/scene/model/animation.cxx: Csaba/Jester : fix a problem
  with tex transform animation

2008-12-12 20:57  fredb

* simgear/scene/model/SGMaterialAnimation.cxx: Csaba/Jester : fix a
  problem with material animation

2008-12-12 08:41  fredb

* simgear/sound/sample_openal.cxx: Print the name of the file
  generating an OpenAL error

2008-12-11 09:24  fredb

* simgear/scene/sky/: newcloud.cxx, newcloud.hxx, sky.cxx, sky.hxx:
  Stuart : - Removes the cloud type re-use code - I think they
  aren't worth the graphical artefacts that they cause in light of
  Tim's improvements - Changes the transparency of the clouds.
  Previously, the clouds were transparent at 0m distance, opaque at
  200m, then gradually more transparent to the fog limits. This
  meant they were generally quite transparent. Now, they are opaque
  from 200m to 15km, then become transparent at 20km. A side effect
  of this is that the current textures could probably benefit from
  being made slightly transparent to improve the blending of the
  sprites against each other.

2008-12-11 09:23  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects. Add
  Boost

2008-12-10 23:39  timoore

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx, newcloud.cxx,
  sky.cxx: Use a singleton Fog attribute for all 3D clouds.

  Don't update this Fog with any kind of update callback; instead,
  update from the sky repaint method.

2008-12-10 23:39  timoore

* simgear/scene/material/mat.cxx: Use StateAttributeFactory for
  CullFace and ShadeModel attributes

2008-12-10 23:39  timoore

* acinclude.m4, configure.ac,
  simgear/scene/model/ModelRegistry.cxx,
  simgear/scene/model/ModelRegistry.hxx,
  simgear/scene/sky/newcloud.cxx,
  simgear/scene/tgdb/GroundLightManager.cxx,
  simgear/scene/tgdb/GroundLightManager.hxx,
  simgear/scene/util/StateAttributeFactory.cxx,
  simgear/scene/util/StateAttributeFactory.hxx,
  simgear/structure/Makefile.am, simgear/structure/Singleton.hxx:
  Use Boost singleton template for our singletons

2008-12-10 23:38  timoore

* simgear/scene/sky/cloud.cxx: Render cloud layers with face
  culling

  Also, allow definition of colors for the cloud layer corners for
  debugging.

2008-12-10 23:38  timoore

* simgear/scene/util/: StateAttributeFactory.cxx,
  StateAttributeFactory.hxx: Add cullFaceFront as a state attribute
  to the StateAttributeFactory

2008-12-10 23:37  timoore

* simgear/scene/sky/sky.cxx: Render sky with depth test off.

  Also, don't set BACKGROUND_BIT for cloud layers.

2008-12-10 19:20  durk

* configure.ac: Finalizing the preparation of SimGear-1.99.5-rc2.

2008-12-07 00:02  fredb

* simgear/scene/sky/: CloudShaderGeometry.cxx,
  CloudShaderGeometry.hxx, cloudfield.cxx, cloudfield.hxx,
  newcloud.cxx, newcloud.hxx: Stuart Buchanan : - Replaces simple
  shader attributes with vectors (this was missed out of the last
  patch by mistake) - Includes Yon's Fog update code (Thanks!) -
  Fixes a bug since 1.0 where --enable-real-weather-fetch stopped
  the other weather scenarios from working.

2008-12-07 00:01  fredb

* simgear/nasal/hash.c: Ugly hack to fix a memory corruption
  problem

2008-12-04 21:56  fredb

* simgear/scene/sky/: CloudShaderGeometry.cxx,
  CloudShaderGeometry.hxx, cloud.cxx, cloud.hxx, cloudfield.cxx,
  cloudfield.hxx, newcloud.cxx, newcloud.hxx, sky.cxx, sky.hxx:
  Stuart Buchanan :

  This provides the following enhancements & bug fixes - Fix the
  chequer-board bug.  - Add proper cloud coverage function - so
  scattered clouds are now truly scattered.  - Add real-time
  control for visibility range.  - Use a limited set of clouds
  rather than generating a completely new Geode for each cloud.
  This saves sorting and display time.	- Add controls to Rendering
  dialog to allow fine-tuning of the number of sprites, cloud
  visibility and the number of different types of cloud.  - Add
  some variance to the sort back-off to avoid all clouds being
  sorted at the same time.  - Pack attributes into vectors for
  performance - Re-order the cloud type determination code so that
  if a cloud layer could either be stratus or cumulus, cumulus is
  used.  - Lowered the cloud level in the standard cloud
  configuration slightly so a cumulus layer is generated rather
  than stratus.

  These last two mean that you should see some 3D cumuli if
  disabling real weather fetch.

  My thanks to Yon Uriarte for his help with performance work.

2008-12-01 00:06  fredb

* simgear/scene/sky/: CloudShaderGeometry.cxx,
  CloudShaderGeometry.hxx, cloud.cxx, cloudfield.cxx,
  cloudfield.hxx, newcloud.cxx: Stuart: Attached is another clouds
  patch. This does the following: 1) Puts the 3D clouds in a cloud
  rendering bin, to reduce the transparent edge problem. Viewing 3d
  clouds against a 2D layer _above_ it now blends correctly. There
  is still a problem when viewing a layer below the 3d clouds, and
  I'm not sure if/how I'll manage to solve that. Thanks to Tim for
  pointing me at the correct code (again).  2) Performance
  improvement by calculating the Bounding box when the cloud is
  generated rather than ever time it is requested.

2008-11-29 01:17  timoore

* simgear/debug/: logstream.cxx, logstream.hxx: logstream
  improvements from Yon Uriarte

  Avoid descending into iostream when a message won't be logged.

2008-11-26 08:28  fredb

* simgear/scene/sky/: cloud.cxx, cloudfield.cxx, cloudfield.hxx,
  CloudShaderGeometry.cxx, CloudShaderGeometry.hxx: Stuart:
  > Something has changed in the environment manager which means
  that clouds
  > generateion is now inconsistent. I'm still tracking it down, as
  my recent
  > changes shouldn't have affected this.

  Well, the cause was a bug in my code, but it didn't expose itself
  until we moved to multiple cameras. The attached patch fixes the
  problem.

  I've also put in a new heuristic to improve the frame-rate.
  Clouds that are already sorted are likely to still be sorted in
  subsequent frames. Therefore I've put in a back-off mechanism for
  the bubble-sort pass. This should mean that if you stay
  completely stationary, once the clouds become sorted they will
  eventually only perform a bubble sort pass every 128 frames.

2008-11-24 23:08  timoore

* simgear/scene/tgdb/: TileCache.cxx, TileCache.hxx: Track time in
  TileCache; assign new tiles the current time

2008-11-24 21:26  fredb

* simgear/scene/sky/: cloudfield.cxx, CloudShaderGeometry.hxx:
  Stuart :
  > Warning: detected OpenGL error 'valeur non valide' after
  RenderBin::draw(,)

  Fixed in the patch below. For some reason the shader didn't like
  index 16 being used...

  The patch also fixes the chequer-board effect that was causing
  very sparse cloud cover.

2008-11-23 13:14  fredb

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx,
  CloudShaderGeometry.cxx, CloudShaderGeometry.hxx, newcloud.cxx:
  Stuart Buchanan : Attached is a small patch for 3D clouds.  It
  provide the following: 1) Proper spherical distribution of
  sprites (previously they were distributed cylindrically - whoops)
  2) Better shading, so the bottom of the cloud is darker than the
  top.	3) Fixed a couple of texture sizing bugs.

2008-11-21 15:48  timoore

* simgear/structure/: Makefile.am, OSGUtils.hxx: Templates for
  interacting with OSG objects

2008-11-18 23:16  timoore

* simgear/props/props.hxx: Template functions and overloaded
  functions for accessing property values.

  These are useful for writing templates that manipulate
  properties.

2008-11-09 16:39  curt

* simgear/scene/sky/CloudShaderGeometry.cxx: Manuel Massing:

  Attached is a small fix for the sorting in
  CloudShaderGeometry.cxx.  I think the sorting problem stems from
  the osg idiosyncracy to store transposed matrices...so the
  intuitive

    osg::Vec4f p = vm *
  osg::Vec4f(_cloudsprites[i]->position.osg(), 1.0f);

  needs to be replaced with...

    osg::Vec4f p =
  vm.preMult(osg::Vec4f(_cloudsprites[i]->position.osg(), 1.0f);

  The patch also optimizes the distance calculation - it evaluates
  the distances in model space instead of eye space, which reduces
  computation to a dot- product instead of a matrix multiplication.

2008-11-06 22:58  fredb

* simgear/scene/sky/: CloudShaderGeometry.cxx,
  CloudShaderGeometry.hxx, cloudfield.cxx, cloudfield.hxx,
  newcloud.cxx, newcloud.hxx, sky.cxx, sky.hxx: Stuart Buchanan :
  It fixes the following issues (to a greater or lesser extent): 1)
  Performance. Quad trees used to improve culling, and the sprites
  are placed on the surface of a sphere rather than randomly
  throughout the cloud, requiring fewer textures. This saves about
  5-10fps on my machine.  2) Disabled 3D clouds have no performance
  impact. Previously they were still in the scenegraph. Now they
  are removed.	3) Clouds are now loaded on start-up, and don't
  require the scenario to be changed, they also work with METAR.
  4) The cloud field is shifted as you travel. There's a small bug
  in that the clouds "jump" as you reach the edge of the field.  5)
  Iterative sorting of sprites. This doesn't appear to solve the
  alpha blending problem completely, but may help a bit.

2008-10-31 10:46  timoore

* simgear/environment/precipitation.cxx: Respect
  precipitation-enable property

  From Csaba Halász

2008-10-31 00:51  timoore

* simgear/scene/model/particles.cxx: enable world coordinate
  particle fix

2008-10-30 18:51  curt

* Doxyfile, NEWS, SimGear.dsp, configure.ac: Tidy up for a source
  code "snapshot" release.

2008-10-26 10:37  fredb

* simgear/scene/sky/newcloud.cxx, simgear/scene/sky/cloudfield.cxx,
  simgear/scene/sky/cloud.cxx, simgear/scene/sky/Makefile.am,
  simgear/scene/sky/cloud.hxx, simgear/scene/sky/cloudfield.hxx,
  simgear/scene/sky/newcloud.hxx, simgear/scene/sky/sky.cxx,
  simgear/scene/sky/sky.hxx, projects/VC7.1/SimGear.vcproj,
  simgear/environment/visual_enviro.cxx,
  simgear/environment/visual_enviro.hxx,
  simgear/scene/sky/CloudShaderGeometry.cxx,
  simgear/scene/sky/CloudShaderGeometry.hxx: 3D clouds from Stuart
  Buchanan. Need a recent driver update, --enable-clouds3d option
  and a Weather Scenario to show up

2008-10-23 20:46  curt

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.hxx, xmlsound.hxx: Fix a few spelling errors.

2008-10-18 20:44  mfranz

* simgear/scene/model/SGReaderWriterXML.cxx: fix regression:
  sgLoad3DModel_internal should not make assumptions about the type
  of SGModelData. This *can* have to do with Nasal, but doesn't
  have to. That's entirely that class instance's business.

2008-10-17 23:15  mfranz

* simgear/scene/model/: particles.cxx, particles.hxx: add global
  switch for particle systems

2008-10-14 15:01  mfranz

* simgear/scene/model/SGReaderWriterXML.cxx: allow to modify the
  XML animation config of a loaded submodel by defining an
  <overlay> branch, which is copied over the submodel's properties
  before the animations are evaluated

2008-10-14 13:03  mfranz

* simgear/props/props_io.cxx: Move omit-node part from startElement
  to endElement to allow nodes of this type to have children. We
  have to save the omit state on the stack for that.

2008-10-14 07:04  durk

* simgear/scene/sky/: cloud.cxx, moon.cxx, newcloud.cxx,
  oursun.cxx: Syd Adams: Replace rgb with png.

2008-10-10 11:48  mfranz

* simgear/scene/model/SGReaderWriterXML.cxx:
  s/multiplayer/multiplay/, as this seems to be the slightly more
  official version (--multiplay option, /sim/multiplay/ properties
  etc.)

2008-10-10 00:20  mfranz

* simgear/scene/model/SGReaderWriterXML.cxx: if a model XML file
  contains a <multiplayer> block, copy its contents  to the model's
  property root (/ai/models/multiplayer[*])

2008-10-03 21:38  ehofman

* simgear/nasal/: bitslib.c, code.c, code.h, codegen.c, data.h,
  gc.c, hash.c, iolib.c, lex.c, lib.c, misc.c, naref.h, nasal.h,
  parse.c, parse.h, string.c, threadlib.c, utf8lib.c, vector.c:
  Sync. w. OSG branch

2008-09-30 23:52  andy

* simgear/nasal/lex.c: Yeah, I really shouldn't be in the lexer

2008-09-30 22:19  andy

* simgear/nasal/lex.c: Another lexer fix

2008-09-30 21:06  andy

* simgear/nasal/lex.c: Fix broken lex.c checkin

2008-09-30 18:48  andy

* simgear/nasal/: code.c, codegen.c, lex.c, string.c: Fixes for
  bugs shaken out in the recent push: die properly for nil indexes
  in slices.  Fix string conversion issue with bare "+" and "-".
  Fix lexing of exponent expressions such that "1e" is not a
  number.

2008-09-28 10:11  fredb

* simgear/scene/sky/cloud.cxx: MINGW patch by Csaba Halasz

2008-09-26 21:18  andy

* simgear/nasal/: code.c, codegen.c: Fix parsing for degenerate
  cases like a[,] a[:] and {:}.  The slicing syntax exposed the
  low-level generators to some new cases.

2008-09-26 20:22  andy

* simgear/nasal/: bitslib.c, code.c, code.h, codegen.c, data.h,
  gc.c, hash.c, iolib.c, lex.c, lib.c, misc.c, naref.h, nasal.h,
  parse.c, parse.h, string.c, threadlib.c, utf8lib.c, vector.c:
  Sync with Nasal upstream (Melchior already had a chance to test
  this, so hopefully not too much breaks).  New syntax features:

  1. Call-by-name function arguments.  You can specify a hash
  literal in place of ordered function arguments, and it will
  become the local variable namespace for the called function,
  making functions with many arguments more readable.  Ex:

      view_manager.lookat(heading:180, pitch:20, roll:0, x:X0, y:Y0,
  z:Z0,
        time:now, fov:55);

  Declared arguments are checked and defaulted as would be
  expected: it's an error if you fail to pass a value for an
  undefaulted argument, missing default arguments get assigned, and
  any rest parameter (e.g. "func(a,b=2,rest...){}") will be
  assigned with an empty vector.

  2. Vector slicing.  Vectors (lists) can now be created from
  others using an ordered list of indexes and ranges.  For example:

      var v1 = ["a","b","c","d","e"]

      var v2 = v1[3,2];	 # == ["d","c"];
      var v3 = v1[1:3];	 # i.e. range from 1 to 3: ["b","c","d"];
      var v4 = v1[1:];	 # no value means "to the end":
  ["b","c","d","e"]
      var i = 2;
      var v5 = v1[i];	 # runtime expressions are fine: ["c"]
      var v6 = v1[-2,-1]; # negative indexes are relative to end:
  ["d","e"]

  The range values can be computed at runtime (e.g. i=1;
  v5=v1[i:]).  Negative indices work the same way the do with the
  vector functions (-1 is the last element, -2 is 2nd to last,
  etc...).

  3. Multi-assignment expressions.  You can assign more than one
  variable (or lvalue) at a time by putting them in a parenthesized
  list:

      (var a, var b) = (1, 2);
      var (a, b) = (1, 2);		# Shorthand for (var a, var
  b)
      (var a, v[0], obj.field) = (1,2,3) # Any assignable lvalue
  works

      var color = [1, 1, 0.5];
      var (r, g, b) = color;  # works with runtime vectors too

2008-09-11 12:19  timoore

* simgear/scene/model/: SGReaderWriterXML.cxx, particles.cxx,
  particles.hxx: Keep non-local particles in a Z-up frame that is
  periodically moved.

  This eliminates jitter and other rendering problems.	For the
  moment this is dependent on an osg fix.

  Also, don't read wind properties from FlightGear; provide a
  mechanism for fg to set the wind.

2008-09-04 10:52  ehofman

* simgear/scene/model/SGReaderWriterXMLOptions.hxx: Make sure the
  compiler is aware that SGModelData a child of osg::Referenced,
  now gcc-3.3 will compile.

2008-08-31 20:39  fredb

* simgear/scene/model/SGPagedLOD.hxx: Use a more relevant library
  name

2008-08-31 20:35  fredb

* simgear/simgear_config.h-msvc71: Update version

2008-08-29 09:39  timoore

* simgear/scene/model/model.cxx: Don't use
  osgDB::SharedStateManager::share in SGLoadTexture2D

  It is not safe to call this function from the database pager
  thread; in any event, state sets and textures created in the
  database pager will get passed through the SharedStateManager
  anyway.

2008-08-28 19:52  curt

* SimGear.dsp, configure.ac, simgear/scene/tgdb/Makefile.am: -
  Update SimGear.dsp - Assign an arbitrary prerelease version
  number - Fix a "make dist" problem.

2008-08-25 18:53  andy

* simgear/nasal/iolib.c: Fix typing error with fgetc in readln().
  On most boxes, this would cause a spurious EOF when there was a
  0xff in the stream.  But on PPC, char is unsigned (for reasons
  known only to IBM) and it would loop forever.

2008-08-11 10:32  timoore

* simgear/scene/model/: particles.cxx, particles.hxx: pointers in
  Particles object should be ref_ptr

2008-08-08 00:24  timoore

* simgear/io/: sg_file.cxx, sg_file.hxx: Return eof after a number
  of reptetitions of file input.

2008-08-02 13:31  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects - Adapt
  to OSG 2.6.0-rc1 : location of header files should now be
  searched in the install directory, here
  ..\..\..\install\msvc71\OpenSceneGraph\include

2008-07-29 10:25  ehofman

* simgear/: compiler.h, io/sg_binobj.cxx, props/props.cxx,
  sound/sample_openal.hxx, structure/subsystem_mgr.hxx: final fixes
  for SG_USING_STD removal

2008-07-29 04:54  timoore

* simgear/scene/sky/dome.hxx: forward declare
  osg::DrawElementsUShort for real

2008-07-28 09:52  ehofman

* simgear/: debug/logstream.hxx, environment/metar.hxx,
  environment/visual_enviro.cxx, environment/visual_enviro.hxx,
  ephemeris/stardata.cxx, io/decode_binobj.cxx, io/iochannel.hxx,
  io/lowtest.cxx, io/sg_file.cxx, io/sg_file.hxx, io/sg_serial.cxx,
  io/sg_serial.hxx, io/sg_socket.hxx, io/sg_socket_udp.hxx,
  io/socktest.cxx, io/tcp_server.cxx, math/interpolater.cxx,
  math/interpolater.hxx, math/sg_types.hxx, misc/sg_path.hxx,
  misc/strutils.hxx, misc/tabbed_values.hxx,
  misc/tabbed_values_test.cxx, misc/texcoord.cxx,
  props/condition.cxx, props/props_io.cxx, props/props_test.cxx,
  route/route.hxx, route/routetest.cxx, route/waypoint.hxx,
  route/waytest.cxx, scene/material/mat.cxx,
  scene/material/mat.hxx, scene/material/matlib.cxx,
  scene/material/matlib.hxx, scene/material/matmodel.cxx,
  scene/model/model.cxx, scene/model/model.hxx,
  scene/model/modellib.hxx, scene/model/shadowvolume.hxx,
  scene/sky/cloud.hxx, scene/sky/cloudfield.cxx,
  scene/sky/cloudfield.hxx, scene/sky/newcloud.hxx,
  scene/sky/sky.hxx, scene/tgdb/TileCache.hxx,
  scene/tgdb/TileEntry.cxx, scene/tgdb/TileEntry.hxx,
  scene/tgdb/apt_signs.cxx, scene/tgdb/apt_signs.hxx,
  scene/tgdb/obj.hxx, scene/tgdb/pt_lights.hxx,
  scene/tgdb/userdata.hxx, screen/shader.h, serial/serial.hxx,
  serial/testserial.cxx, sound/soundmgr_openal.hxx,
  structure/commands.hxx, structure/exception.hxx, xml/easyxml.cxx,
  xml/easyxml.hxx: Replace SG_USE_STD() by using std::

2008-07-27 20:04  fredb

* simgear/screen/extensions.hxx: GL/wgl.h never existed, and a glx
  equivalent is not needed

2008-07-27 18:10  ehofman

* simgear/: compiler.h, scene/model/shadowvolume.cxx,
  scene/sky/bbcache.cxx, scene/sky/newcloud.cxx,
  screen/GLBitmaps.h, screen/RenderTexture.cpp,
  screen/RenderTexture.h, screen/TestRenderTexture.cpp,
  screen/extensions.hxx, screen/screen-dump.cxx,
  screen/screen-dump.hxx, screen/texture.cxx, screen/texture.hxx,
  screen/tr.cxx, screen/tr.h:

    - remove the SG_GLxxxx_H #defines, since OSG provides its own
  versions
    - this exposed a bizarre issue on Mac where dragging in
  <AGL/agl.h> in
      extensions.hxx was pulling in all of Carbon to the global
  namespace
      - very scary. As a result, I now need to explicitly include
  CoreFoundation
      in fg_init.cxx.
    - change SG_USING_STD(x) to using std::x

  Issues:

    - the logic for X11 and Win32 in RenderTexture and extensions is
  tortured,
      please see if you agree I got all the ifdefs correct.

2008-07-27 17:15  fredb

* simgear/: scene/sky/dome.cxx, scene/material/matlib.cxx,
  screen/RenderTexture.cpp, structure/subsystem_mgr.hxx,
  timing/timestamp.cxx: Remove unneeded inclusions of windows.h,
  GL.h and GLU.H

2008-07-25 23:33  fredb

* simgear/misc/zfstream.hxx: Maybe a more portable way to do it

2008-07-25 23:27  fredb

* simgear/misc/zfstream.hxx: Compile again with MSVC

2008-07-25 20:35  ehofman

* simgear/: Makefile.am, compiler.h, constants.h, sg_traits.hxx,
  debug/logstream.hxx, environment/visual_enviro.hxx,
  io/decode_binobj.cxx, io/iochannel.hxx, io/lowtest.cxx,
  io/sg_binobj.cxx, io/sg_binobj.hxx, io/sg_file.cxx,
  io/sg_serial.cxx, io/sg_serial.hxx, io/sg_socket.cxx,
  io/sg_socket.hxx, io/sg_socket_udp.hxx, io/socktest.cxx,
  io/tcp_client.cxx, io/tcp_server.cxx, magvar/coremag.cxx,
  math/SGCMath.hxx, math/SGMath.hxx, math/interpolater.cxx,
  math/interpolater.hxx, math/sg_types.hxx, misc/sg_path.hxx,
  misc/sgstream.cxx, misc/sgstream.hxx, misc/strutils.hxx,
  misc/tabbed_values.hxx, misc/tabbed_values_test.cxx,
  misc/texcoord.cxx, misc/zfstream.cxx, misc/zfstream.hxx,
  props/condition.cxx, props/props_io.cxx, props/props_io.hxx,
  props/props_test.cxx, route/routetest.cxx, route/waypoint.hxx,
  route/waytest.cxx, scene/material/mat.hxx,
  scene/material/matlib.cxx, scene/material/matlib.hxx,
  scene/material/matmodel.hxx, scene/model/ModelRegistry.hxx,
  scene/model/modellib.hxx, scene/sky/cloud.hxx,
  scene/sky/cloudfield.cxx, scene/sky/moon.cxx,
  scene/sky/newcloud.cxx, scene/sky/newcloud.hxx,
  scene/sky/sphere.cxx, scene/sky/stars.cxx,
  scene/tgdb/TileCache.cxx, scene/tgdb/TileEntry.cxx,
  scene/tgdb/TileEntry.hxx, scene/tgdb/apt_signs.hxx,
  scene/tgdb/obj.hxx, scene/tgdb/pt_lights.hxx,
  scene/tgdb/userdata.hxx, screen/shader.h, serial/serial.cxx,
  serial/serial.hxx, serial/testserial.cxx,
  sound/sample_openal.hxx, sound/soundmgr_openal.cxx,
  sound/soundmgr_openal.hxx, structure/commands.hxx,
  structure/exception.hxx, structure/subsystem_mgr.hxx,
  threads/SGQueue.hxx, threads/SGThread.hxx, timing/geocoord.h,
  timing/sg_time.cxx, timing/sg_time.hxx, timing/timestamp.cxx,
  timing/timezone.h, xml/easyxml.cxx, xml/easyxml.hxx:
  Reduce compiler.h to almost nothing (but it's worth keeping
  around I think, for the MSVC and MipsPro warning stuff).

  As a result of this patch, simgear/sg_traits.h can be deleted. So
  can SGCMath.h, but I'll do that separately.

  There is one more 'mechanical' change to come - getting rid of
  SG_USING_STD(X), but I want to keep that separate from everything
  else. (There's another mechnica l change, replacing <math.h> with
  <cmath> and so on *everywhere*, but one step a t a time)

2008-07-25 12:39  ehofman

* simgear/: compiler.h, constants.h, ephemeris/celestialBody.cxx,
  ephemeris/jupiter.cxx, ephemeris/mars.cxx, ephemeris/mercury.cxx,
  ephemeris/moonpos.cxx, ephemeris/saturn.cxx, ephemeris/star.cxx,
  ephemeris/uranus.cxx, ephemeris/venus.cxx, math/point3d.hxx,
  misc/sgstream.hxx, scene/material/mat.cxx,
  scene/material/matlib.cxx, scene/material/matmodel.cxx:
  Attached patches remove BORLANDC, and hence
  SG_MATH_EXCEPTION_CLASH and SG_INCOM PLETE_FUNCTIONAL from
  SimGear and FlightGear.

  As a result, SG_HAVE_STD_INCLUDES is now *always* set, so I will
  get the boring fixes for that done, but separately. I'm still
  auditing the other things in comp ilers.h - there's a lot that
  can die now BORLAND is gone.

2008-07-25 10:34  ehofman

* simgear/debug/logstream.cxx:
  remove a workaround for the C++ / stdlibary bug which existed
  aeons ago on Mac.

2008-07-24 21:16  ehofman

* simgear/: compiler.h, debug/logstream.hxx, misc/sg_path.cxx,
  misc/sgstream.cxx, misc/zfstream.cxx, screen/colors.hxx,
  timing/lowleveltime.cxx, timing/timestamp.cxx:
  Patch to remove macintosh and MWERKS from Simgear.

2008-07-19 18:01  timoore

* simgear/io/: sg_file.cxx, sg_file.hxx: Add repeat option to
  SGFile.

  This supports auto-looping over a playback file, for demos and
  such.

2008-07-19 18:00  timoore

* simgear/scene/tgdb/: SGReaderWriterBTG.cxx,
  SGReaderWriterBTGOptions.hxx: Eliminate empty default constructor
  for SGReaderWriterBTGOptions

2008-07-12 17:31  mfranz

* simgear/scene/model/particles.cxx: s/getChild/getNode/
  (otherwise "start/size" wouldn't work)

2008-07-12 16:06  mfranz

* simgear/scene/model/: particles.cxx, particles.hxx: - XML
  interface changes:   * condition at top level   *
  <start|end>/<color|size> hierarchy   * wind/gravity -> bool	*
  rename some properties (rotspeed -> rotation-speed, etc.)   *
  unit suffixes - remove redundant code - warnings--

2008-07-10 12:33  mfranz

* simgear/props/props.cxx: fix SGPropertyNode::LAST_USED_ATTRIBUTE

2008-07-09 19:31  mfranz

* simgear/props/props_io.cxx: write-protection warning: use
  simplified path

2008-07-09 18:17  mfranz

* simgear/props/props_io.cxx: let readProperties() refuse to
  overwrite write-proteced properties

2008-06-22 20:07  mfranz

* simgear/debug/logstream.hxx: remove redundant #defines (they are
  already in compiler.h)

2008-06-12 10:14  timoore

* simgear/scene/: model/ModelRegistry.cxx, model/animation.cxx,
  model/shadanim.cxx, tgdb/obj.cxx, tgdb/pt_lights.cxx: From Benoit
  Laniel: replace SG threading constructs with those from
  OpenThreads

  Also, move any static local mutexes up to global level.

2008-06-08 18:45  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-06-07 16:45  mfranz

* simgear/environment/: metar.cxx, metar.hxx: warnings--

2008-06-02 22:26  timoore

* projects/VC7.1/SimGear.vcproj: add OSGVersion.hxx to VC7.1
  project file

2008-06-02 22:22  timoore

* simgear/: scene/model/SGPagedLOD.cxx, scene/tgdb/TileEntry.hxx,
  structure/Makefile.am, structure/OSGVersion.hxx: OSG
  DatabasePager interface change in 2.5.1

  Move OSG version macro from FlightGear to simgear

2008-06-02 22:21  timoore

* simgear/: props/props_io.hxx, structure/SGSmplhist.hxx: Replace
  header files istream and ostream with iosfwd

2008-06-02 22:21  timoore

* simgear/: bucket/newbucket.hxx, debug/logstream.cxx,
  debug/logstream.hxx, io/sg_binobj.cxx, io/tcp_client.cxx,
  math/point3d.hxx, misc/sgstream.cxx, misc/sgstream.hxx,
  misc/zfstream.cxx, misc/zfstream.hxx, props/props.cxx,
  props/props.hxx, props/props_io.cxx, props/props_io.hxx,
  props/props_test.cxx, scene/material/matmodel.hxx,
  scene/model/shadanim.cxx, scene/tgdb/TileEntry.cxx,
  sound/xmlsound.cxx, sound/xmlsound.hxx, structure/SGSmplhist.hxx:
  Don't include <iostream> and "using" declarations in header files

  <iostream> sucks in expensive initialization of the standard
  streams and isn't appropriate in a header file. Use <istream> and
  <ostream> instead.

  using declarations should never appear at global scope in a
  header file; source files get to decide what they want to use in
  their namespace.

2008-06-01 19:25  fredb

* simgear/scene/util/SGSceneFeatures.cxx: Fix non threadsafe code

2008-05-31 14:08  fredb

* simgear/scene/model/SGReaderWriterXML.cxx: Report abnormal
  condition

2008-05-31 14:06  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-05-31 14:05  fredb

* simgear/scene/model/SGPagedLOD.hxx: Fix a problem in loading
  models with paged LOD in some circunstances ( debug with MSVC for
  instance )

2008-05-31 12:02  fredb

* simgear/: scene/material/mat.hxx, scene/material/mat.cxx,
  scene/tgdb/TileEntry.cxx, sound/sample_openal.cxx: Use bool
  instead of int to represent boolean values

2008-05-21 16:51  mfranz

* simgear/sound/xmlsound.cxx: xmlsound: warning--

2008-05-21 16:47  mfranz

* simgear/sound/: xmlsound.cxx, xmlsound.hxx: add <delay-sec>
  parameter that defines how many seconds after triggering the
  sound should be played (default: 0)

2008-05-19 17:17  mfranz

* README.OSG, README.plib: update library version requirements

2008-05-15 08:19  fredb

* simgear/scene/tgdb/: ReaderWriterSTG.cxx, TileCache.cxx,
  TileEntry.cxx: Win32 fix

2008-05-15 00:07  timoore

* projects/VC7.1/SimGear.vcproj,
  simgear/scene/model/ModelRegistry.cxx,
  simgear/scene/model/SGReaderWriterXML.cxx,
  simgear/scene/tgdb/Makefile.am,
  simgear/scene/tgdb/ReaderWriterSTG.cxx,
  simgear/scene/tgdb/ReaderWriterSTG.hxx,
  simgear/scene/tgdb/SGReaderWriterBTGOptions.hxx,
  simgear/scene/tgdb/TileCache.cxx,
  simgear/scene/tgdb/TileCache.hxx,
  simgear/scene/tgdb/TileEntry.cxx,
  simgear/scene/tgdb/TileEntry.hxx,
  simgear/scene/tgdb/userdata.cxx: sg: move most scenery-related
  code to simgear

  From Till Busch

2008-05-01 14:21  mfranz

* simgear/scene/tgdb/obj.cxx: Till BUSCH: enable HOT intersection
  tests on random buildings

2008-04-26 17:25  mfranz

* simgear/misc/: sg_path.cxx, sg_path.hxx: Nicolas: let
  SGPath::create_dir() return success/failure (for screenshot)

2008-04-25 10:41  mfranz

* simgear/scene/model/SGPagedLOD.cxx: Till BUSCH:

  "tim recently noticed the database pager was repeatedly loading
  and unloading the same objects. he also tracked down the problem
  to missing bounding sphere information in osgDB::PagedLOD. this
  is a simplicistic approach to fix this: SGPagedLOD will now
  remember whatever value it sees for getBound() after loading a
  child. this patch will make database pager run much calmer."

2008-04-25 00:06  timoore

* simgear/scene/: model/ModelRegistry.cxx, model/ModelRegistry.hxx,
  util/NodeAndDrawableVisitor.hxx: Fix new livery code

  It turns out that the database pager causes the texture image to
  be unloaded after it is applied, so the image and its file name
  may not be available for doing the livery substitution. Ask a
  work around we set the name of the texture to its file name.

2008-04-24 00:09  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-04-23 20:13  timoore

* projects/VC7.1/SimGear.vcproj,
  simgear/scene/model/ModelRegistry.cxx,
  simgear/scene/util/Makefile.am,
  simgear/scene/util/NodeAndDrawableVisitor.cxx,
  simgear/scene/util/NodeAndDrawableVisitor.hxx,
  simgear/screen/TestRenderTexture.cpp: Rewrite livery texture
  replacement to copy StateSet objects

  Also, add a NodeAndDrawableVisitor that descends into
  osg::Drawable.

  That motivation for this is that it's a bad idea to modify state
  sets that the osgDB::SharedStateManager might be keeping.

2008-04-23 14:28  mfranz

* simgear/scene/model/animation.cxx: Till BUSCH:

  this is a small (-1/+3) patch to fix pick animations on scenery
  objects.  since picking apparently doesn't care for polygon
  offsets, the objects got into the picklist in the wrong way. now,
  no matter if the "highlight group" or the "normal group" gets hit
  first, the callback will fire.

2008-04-14 23:44  timoore

* simgear/: math/Makefile.am, math/Math.hxx, scene/sky/dome.cxx,
  scene/sky/dome.hxx: rewrite of sky dome code

  Add more points to the dome, giving it a dome shape rather than a
  dunce cap shape.

  Represent as OpenGL DrawElements instead of as triangle strips.

  Only calculate have the sky colors and reflect those across the
  dome.

2008-04-14 08:27  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-04-13 23:11  timoore

* simgear/: io/sg_file.cxx, io/sg_serial.cxx, misc/strutils.cxx,
  misc/tabbed_values.cxx, screen/RenderTexture.cpp,
  screen/extensions.cxx, screen/extensions.hxx, screen/shader.cpp,
  structure/SGExpression.cxx: Fixes for compiling with gcc 4.3

  Include standard header files and qualify with std:: where
  needed.

  Add "const" to various char parameters and variables.

2008-04-03 00:25  fredb

* simgear/scene/model/SGReaderWriterXML.cxx: Allows to load
  submodels with path relative to current model path.  Submodel
  path must be prefixed by ./ otherwise path is relative to fg_root
  ( current behavior )

2008-03-24 22:41  timoore

* simgear/scene/tgdb/obj.cxx: Change the tile light group node mask
  to traverse VASI lights too.

  The change to set the light group node mask to LIGHTS_BITS caused
  VASI/PAPI lights to not be displayed during the day. This patch
  fixes that and optimizes VASI creation a bit.

2008-03-22 12:39  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-03-22 12:38  fredb

* simgear/scene/model/SGReaderWriterXML.cxx: Win32 fix

2008-03-22 10:30  timoore

* simgear/: constants.h, scene/material/matmodel.cxx,
  scene/material/matmodel.hxx, scene/model/CheckSceneryVisitor.cxx,
  scene/model/CheckSceneryVisitor.hxx, scene/model/Makefile.am,
  scene/model/SGPagedLOD.cxx, scene/model/SGPagedLOD.hxx,
  scene/model/SGReaderWriterXML.cxx,
  scene/model/SGReaderWriterXML.hxx,
  scene/model/SGReaderWriterXMLOptions.hxx, scene/model/model.cxx,
  scene/model/model.hxx, scene/model/modellib.cxx,
  scene/model/modellib.hxx, scene/tgdb/userdata.cxx,
  scene/tgdb/userdata.hxx, structure/subsystem_mgr.hxx: model
  paging patch from Till Busch

  Comments from Till: I started the project at the end of february
  with a simple idea: move all 3d-model loading to the
  DatabasePager-thread. my first attempts looked promising, though
  they were a little too optimistic (or naive?). the patch has
  evolved a lot since.

  currently it does the following things: 1. revive SGModelLib,
  move functions for xml-model-loading there

  2. replace all calls to sgLoad3dModel with calls to either
  SGModelLib::loadModel() or SGModelLib::loadPagedModel() almost
  all models will be loaded by the DatabasePager. the few
  exceptions are: your own plane, shared models in scenery, random
  objects, AIBallistic models.

  3. simplify mode-loading functions (avoid passing around fg_root)

  4. avoid supurious MatrixTransform nodes in loaded models

  5. fix some memory leaks

2008-03-20 18:20  mfranz

* simgear/sound/xmlsound.cxx: - warnings-- - make one-shot sounds
  subject to volume and pitch control (To get	constant
  volume/pitch during the whole lifetime, just *configure*   the
  sound that way.)

2008-03-17 09:46  timoore

* simgear/scene/tgdb/obj.cxx: rationalize node masks a bit

2008-03-15 17:33  curt

* configure.ac: argh, missed 1.8.4 in one spot.

2008-03-15 17:32  curt

* configure.ac: Simgear also enforces plib-1.8.5 in order to build.

2008-03-11 14:44  timoore

* acinclude.m4: From Till Busch: use install -p if possible

2008-03-09 23:09  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-03-04 12:48  mfranz

* simgear/scene/tgdb/apt_signs.cxx: use new arrow shortcuts for
  compatibility reasons (will be removed later)

2008-03-04 09:58  timoore

* simgear/environment/: precipitation.cxx, precipitation.hxx:
  cleanup of precipitation contribution

  Reindent everything to Stroustrup style and make member variable
  style consistent.

  Remove unused header files.

  SGPrecipitation is now a subclass of osg::Referenced.

  Initialize snow and ice intensity to 0 directly. The methods that
  set the intensities change the value slowly and so don't work
  when the initial value is garbage.

2008-03-04 09:54  timoore

* simgear/environment/: Makefile.am, precipitation.cxx,
  precipitation.hxx: precipitation effects from Nicolas Vivien

2008-03-04 09:53  timoore

* simgear/math/: Makefile.am, SGGeod.cxx, SGGeod.hxx: Add methods
  to SGGeod to return OSG Matrix objects for local frames.

  Methods have been added for Z down (simulation) and Z up frames.

2008-03-02 17:49  fredb

* simgear/scene/model/particles.cxx: Initialize variables before
  using them

2008-02-28 22:25  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-02-23 10:08  mfranz

* simgear/environment/metar.cxx: don't reset modifier in M1SM-type
  visibility

2008-02-21 00:36  mfranz

* simgear/environment/metar.cxx: don't reset visibility modifier in
  the M5SM case

2008-02-16 23:09  mfranz

* simgear/scene/material/mat.cxx: Syd ADAMS: remove season suffix,
  this is now done via <condition>s

2008-02-16 18:01  mfranz

* simgear/scene/model/SGMaterialAnimation.cxx: remove
  getRootNode(), as this resets the model root

2008-02-15 18:54  mfranz

* simgear/scene/material/: matlib.cxx, matlib.hxx: Csaba HALASZ:
  implement conditional (e.g. seasonal) texture loading at startup

2008-02-15 07:44  timoore

* simgear/scene/model/: model.cxx, particles.cxx, particles.hxx:
  Cleanup of particles contribution

  Put particles classes inside simgear namespace

  Refactored some redundant code

  Reworked local frame math to use OSG math classes as much as
  possible

2008-02-15 07:44  timoore

* simgear/scene/model/: Makefile.am, animation.cxx, animation.hxx,
  model.cxx, particles.cxx, particles.hxx: particles from Tiago_G

2008-02-08 00:01  timoore

* simgear/scene/tgdb/TreeBin.cxx: Small cleanup of tree shader

2008-02-08 00:01  timoore

* simgear/scene/: tgdb/ShaderGeometry.cxx, tgdb/TreeBin.cxx,
  tgdb/obj.cxx, util/RenderConstants.hxx: Minor cleanup of Stuart
  Buchanan's tree patch.

  Separate random objects and random trees for real.

2008-02-08 00:00  timoore

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  material/matmodel.cxx, tgdb/SGReaderWriterBTG.cxx,
  tgdb/SGReaderWriterBTGOptions.hxx, tgdb/ShaderGeometry.cxx,
  tgdb/ShaderGeometry.hxx, tgdb/TreeBin.cxx, tgdb/TreeBin.hxx,
  tgdb/obj.cxx, tgdb/obj.hxx: Second trees patch from Stuart
  Buchanan

  Adds random variation of tree texture maps

2008-02-03 00:01  timoore

* simgear/scene/: tgdb/ShaderGeometry.cxx, tgdb/ShaderGeometry.hxx,
  tgdb/TreeBin.cxx, tgdb/TreeBin.hxx, tgdb/obj.cxx,
  util/QuadTreeBuilder.cxx, util/QuadTreeBuilder.hxx,
  util/RenderConstants.hxx, util/StateAttributeFactory.cxx,
  util/StateAttributeFactory.hxx, util/VectorArrayAdapter.hxx:
  Cleanup and performance tuning of the random trees code.

  The QuadTreeBuilder class was completely revamped as a templated
  class to support flexible creation of scene graph quad trees, and
  a major bug was fixed as well. Now it actually generates
  quadtrees instead of some weird striped thing.

  One StateSet is shared among all the "forests." The trees are
  drawn after normal terrain objects to minimize some of the
  transparency related artifacts.

  Lighting was implemented in the ShaderGeometry shader (for both
  polygon sides). Ambient-diffuse values for trees are hard-coded
  in TreeBin.cxx.

  DotOsg wrappers were added for ShaderGeometry so it can be output
  in the scene graph dump.

2008-02-03 00:01  timoore

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  material/matmodel.cxx, material/matmodel.hxx, tgdb/Makefile.am,
  tgdb/SGModelBin.hxx, tgdb/ShaderGeometry.cxx,
  tgdb/ShaderGeometry.hxx, tgdb/TreeBin.cxx, tgdb/TreeBin.hxx,
  tgdb/obj.cxx, util/QuadTreeBuilder.cxx, util/QuadTreeBuilder.hxx:
  Random trees from Stuart Buchanan

  Stuart's new file SGTreeBin.hxx has been split into 4 files:
  TreeBin.[ch]xx and ShaderGeometry.[ch]xx.

2008-01-25 00:05  timoore

* simgear/scene/material/: mat.cxx, mat.hxx, matlib.cxx,
  matlib.hxx: Memory leak fixes from Till Busch

2008-01-21 22:12  curt

* NEWS: At least update the version and date, we can fill in with
  more changes as we have time later.

2008-01-17 22:41  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-01-17 09:28  timoore

* simgear/scene/tgdb/: SGTexturedTriangleBin.hxx, obj.cxx: fix
  memory leaks in random object code

  Don't allocate mt structures (for the random number generator) on
  the heap.

2008-01-12 09:50  fredb

* simgear/scene/model/ModelRegistry.cxx: Avoid spitting gazillion
  'Cannot find image file "" ' message on the console while loading
  random objects

2008-01-07 23:05  frohlich

* simgear/scene/model/animation.cxx: Modified Files:
  simgear/scene/model/animation.cxx:	  Change factor/offset
  order for texture animations.      Fixes remaining bug with
  texture animations.

2008-01-06 16:04  timoore

* simgear/: math/sg_random.c, math/sg_random.h,
  scene/material/matmodel.cxx, scene/material/matmodel.hxx,
  scene/tgdb/SGModelBin.hxx, scene/tgdb/SGTexturedTriangleBin.hxx,
  scene/tgdb/obj.cxx, scene/tgdb/userdata.cxx,
  scene/tgdb/userdata.hxx, scene/util/Makefile.am,
  scene/util/QuadTreeBuilder.cxx, scene/util/QuadTreeBuilder.hxx:
  Random object support from Stuart Buchanan

  In addition to Stuart's changes, there's an independent quad tree
  builder class for constructing loose quad trees from scene graph
  nodes.

  Stuart also implemented changes to the random number generator
  suggested by Andy Ross.

2008-01-04 22:45  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2008-01-04 22:45  fredb

* simgear/structure/SGExpression.cxx: Remove warnings

2008-01-04 08:33  timoore

* simgear/scene/: sky/sky.cxx, sky/sky.hxx,
  util/RenderConstants.hxx: Give the sky a BACKGROUND_BIT nodemask

  Add a MODEL_BIT and tag clouds with that.

  Remove vestigial post_root from sky code.

2008-01-04 08:33  timoore

* simgear/scene/util/RenderConstants.hxx: background node mask

2007-12-31 16:49  frohlich

* simgear/scene/model/: SGClipGroup.cxx, SGClipGroup.hxx: Modified
  Files:       simgear/scene/model/SGClipGroup.cxx
  simgear/scene/model/SGClipGroup.hxx Update the clip group.

2007-12-31 16:48  frohlich

* simgear/scene/model/animation.cxx: Modified Files:
  simgear/scene/model/animation.cxx: Create animation inputs if not
  already there. Fixes broken texture animations from past
  checkin.

2007-12-26 20:19  frohlich

* simgear/scene/util/: Makefile.am, SGPickCallback.hxx: Modified
  Files:	 Makefile.am SGPickCallback.hxx: add proirities for
  pick callbacks.

2007-12-26 20:12  frohlich

* simgear/scene/model/: animation.cxx, animation.hxx: Modified
  Files:   animation.cxx animation.hxx: Make use of the expression
  stuff.	  Factors out common code with other animations
  stuff in flightgear.

2007-12-26 20:10  frohlich

* simgear/structure/: Makefile.am, SGExpression.cxx,
  SGExpression.hxx: Modified Files:  Makefile.am Added Files:
  SGExpression.cxx SGExpression.hxx: Add not yet complete but
  already     usable expression tree. Will be used with the panel
  code.

2007-12-26 20:08  frohlich

* simgear/scene/model/SGClipGroup.cxx: Modified Files:
  scene/model/SGClipGroup.cxx: Oops, add missing simgear_config.h

2007-12-26 20:05  frohlich

* simgear/scene/model/: Makefile.am, SGClipGroup.cxx,
  SGClipGroup.hxx: Modified Files:  Makefile.am Added Files:
  SGClipGroup.cxx SGClipGroup.hxx: Add helper group node for reuse
  of clipping planes. Will be used for the panel code.

2007-12-26 20:03  frohlich

* simgear/props/props.hxx: Modified Files:
  simgear/props/props.hxx: Add a bunch of convinience functions.

2007-12-24 00:33  timoore

* simgear/scene/model/: SGMaterialAnimation.cxx,
  SGMaterialAnimation.hxx, animation.cxx: Pass ReaderWriter options
  to SGMaterialAnimation

  It needs to grab the path list from options in order to support
  the texture change animation.

2007-12-23 00:01  timoore

* simgear/scene/sky/oursun.cxx: Fix tiny bugs in sun constructor
  and repaint code

2007-12-21 07:29  timoore

* projects/VC7.1/SimGear.vcproj: Add StateAttributeFactory files to
  VS project file

2007-12-21 07:25  timoore

* simgear/scene/model/ModelRegistry.cxx: optimize groups from .AC
  file optimizer run

2007-12-21 07:25  timoore

* simgear/scene/: sky/cloud.cxx, util/StateAttributeFactory.cxx,
  util/StateAttributeFactory.hxx: Repaint 2D cloud layers using
  texture combiner

  Don't change the color in the cloud layer geometry

2007-12-21 07:24  timoore

* simgear/scene/: sky/cloud.cxx, tgdb/pt_lights.cxx,
  util/Makefile.am, util/RenderConstants.hxx,
  util/StateAttributeFactory.cxx, util/StateAttributeFactory.hxx:
  Fix cloud layer - point lights visibility issue

  Move point lights to render bin 8, clouds to render bin 9. Turn
  on AlphaFunc for cloud layers.

  Create a StateAttributeFactory object to create and share common
  state objects.

2007-12-18 23:25  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2007-12-14 00:30  timoore

* simgear/scene/: tgdb/GroundLightManager.cxx,
  tgdb/GroundLightManager.hxx, tgdb/Makefile.am, tgdb/obj.cxx,
  util/RenderConstants.hxx: Use node masks and shared state sets to
  manage ground lights

  Do away with the switch in each terrain tile for the ground
  lights. They are turned on by node masks now.

  Share state sets among all the light nodes and manage the fog
  values through a "GroundLightManager" instead of having separate
  state sets and callback functions for each group in each tile.

2007-12-12 18:38  timoore

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Backport of cloud layer
  texture origin fix

  From the OSG commit message:	   Don't reset the random texture
  base when rebuilding a cloud layer	 This fix removes obnoxious
  visuals (texture jumping) when a cloud     layer is moved due to
  a metar update or, more significantly, when	  switching from
  metar to a scenario.

  With help from Vivian Meazza

2007-12-11 22:28  durk

* configure.ac: New version number.

2007-12-11 12:07  timoore

* simgear/: misc/PathOptions.cxx, scene/model/ModelRegistry.cxx:
  minor fix to ModelRegistry and syntax changes for Windows

  Create the local path in the right order in OptionsPusher. When
  OptionsPusher is used, put it inside a new code block so the
  order of destruction with respect to the mutex on reader
  functions id clear.

  Add #include <algorithm> to top of ModelRegistry.cxx.

  Change include syntax in PathOptions.cxx

2007-12-10 09:30  timoore

* simgear/scene/model/ModelRegistry.cxx: Check for null options
  before changing path in registry

2007-12-09 23:38  timoore

* simgear/scene/model/ModelRegistry.cxx: Work around osg Registry
  path list problems

  The OSG reader plugins overwrite the path list passed in options
  with the local directory of the file being read, forcing you to
  set the path list in the Registry. I think this a bug, but in the
  meantime here's a workaround.

2007-12-08 12:07  fredb

* simgear/scene/sky/cloudfield.cxx: Fix a typo

2007-12-08 00:35  timoore

* simgear/bucket/newbucket.hxx: Add operator!= to SGBucket

2007-12-07 10:13  timoore

* simgear/scene/model/: model.cxx, model.hxx: Add option to
  SGLoadTexture2D to load dynamic textures

2007-12-06 18:57  mfranz

* simgear/props/: condition.cxx: - comparison: don't crash if
  second element is missing - better messages ("panel"?!)

2007-12-04 23:38  timoore

* projects/VC7.1/SimGear.vcproj, simgear/misc/Makefile.am,
  simgear/misc/PathOptions.cxx, simgear/misc/PathOptions.hxx,
  simgear/scene/material/mat.cxx,
  simgear/scene/model/ModelRegistry.cxx,
  simgear/scene/model/ModelRegistry.hxx,
  simgear/scene/model/animation.cxx,
  simgear/scene/model/animation.hxx, simgear/scene/model/model.cxx,
  simgear/scene/model/model.hxx, simgear/scene/model/shadanim.cxx,
  simgear/scene/sky/cloud.cxx, simgear/scene/sky/moon.cxx,
  simgear/scene/sky/oursun.cxx: Don't modify OSG Registry with file
  path

  To set a path when loading model files, use an osg
  ReaderWriter::Options object.

  Put locks in ModelRegistry::readNode and ModelRegistry::readImage
  to avoid conflicts when files are loaded from both the pager and
  the main thread.

2007-12-03 17:38  mfranz

* simgear/scene/model/shadowvolume.cxx: make the "noshadow" prefix
  SG_ALERT an SG_WARN

2007-12-03 13:46  mfranz

* simgear/scene/model/shadowvolume.cxx: print full object name in
  noshadow deprecation error message

2007-12-03 11:46  mfranz

* simgear/scene/model/shadowvolume.cxx: let use of deprecated
  "noshadow" prefix cause error message

2007-12-02 13:28  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2007-12-02 10:33  durk

* simgear/scene/sky/cloudfield.cxx: Torsten Dreyer: Make sure 3D
  clouds cache never gets set to zero, thereby	preventing a
  program crash that could occur when switching between OSG and
  plib versions.

2007-11-30 00:56  timoore

* simgear/scene/model/ModelRegistry.hxx: Change the main
  ModelRegistry callback function to stash the substituted file in
  the cache.

2007-11-30 00:56  timoore

* simgear/scene/model/animation.cxx: Avoid copying drawables and
  dirtying display lists.

  For the alpha-test animation, use an OVERRIDE attribute on the
  state set of the top level node instead of copying drawables and
  state sets throughout the model.

  As a temporary hack in the blend animation, don't use display
  lists in the cloned drawables.

  These changes are aimed at cutting down the number of display
  lists that the pager needs to compile.

2007-11-30 00:56  timoore

* simgear/scene/: model/ModelRegistry.cxx, model/ModelRegistry.hxx,
  tgdb/SGReaderWriterBTG.cxx: rewrite ModelRegistry callbacks as a
  template with pluggable policy classes

  In a big effort to improve use of the object cache, provide a
  ModelRegistryCallback template class with different policies for
  substitution,  caching, optimization, etc.

  Change SGTexDataVarianceVistor to make StateSets static too.

2007-11-30 00:55  timoore

* simgear/scene/util/: Makefile.am, RenderConstants.hxx,
  SGNodeMasks.hxx: Start of cleanup of scene graph node masks

2007-11-30 00:55  timoore

* simgear/scene/: model/Makefile.am, model/ModelRegistry.cxx,
  model/ModelRegistry.hxx, model/model.cxx,
  tgdb/SGReaderWriterBTG.cxx, tgdb/SGReaderWriterBTG.hxx: Move
  SGReadFileCallback from model.cxx to public class ModelRegistry

  Move SGReadFileCallback and all its help classes into a new
  ModelRegistry class that also provides an interface to add custom
  callbacks for specific file extensions. SGReaderWriterBTG uses
  that to keep any further processing from being done on .btg
  files. Various namespace-releated cleanup was done on this code
  too.

2007-11-27 20:27  durk

* simgear/: io/sg_file.cxx, io/sg_serial.cxx, misc/strutils.cxx,
  misc/tabbed_values.cxx, scene/model/shadowvolume.cxx,
  screen/RenderTexture.cpp, screen/TestRenderTexture.cpp,
  screen/shader.cpp: Ladislav Michnovič : Compatibility fixes for
  gcc 4.3 Tatsuhiro Nishioka   : Fix shadow rendering for Mac
  Platforms.

2007-11-25 09:27  durk

* simgear/screen/RenderTexture.cpp: Hans Fugal: Two typecasts added
  for OSX Leopard (20.5) compilation.

2007-11-22 19:15  durk

* Makefile.am, SimGear.dsp, configure.ac: A few prerelease related
  changes:  - new version number  - modified path for make dist
  post processing  - new autogenerated dsp file

2007-11-19 00:31  timoore

* simgear/scene/: model/animation.cxx, tgdb/obj.cxx: StateSet
  optimizations

  Use only one shared StateSet to control GL_NORMALIZE. This
  removes thousands of state sets from the scene graph.

  Fix a typo that was causing two copies of groundLights0 to be
  added to each tile.

2007-11-19 00:30  timoore

* simgear/scene/tgdb/: SGTexturedTriangleBin.hxx, obj.cxx: Some
  scene graph optimizations

  When loading terrain, use DrawElementsUShort where possible.

  Don't chunk unconnected triangles in the terrain into separate
  Geometry sets; make the sets as big as possible.

2007-11-18 15:22  durk

* README.OpenAL, README.plib: Minor documentation update.

2007-11-18 15:21  durk

* README.OpenAL, README.plib: Minor documentation updates.

2007-11-17 10:16  durk

* simgear/structure/: Makefile.am, SGSmplhist.cxx, SGSmplhist.hxx,
  SGSmplstat.cxx, SGSmplstat.hxx, subsystem_mgr.cxx,
  subsystem_mgr.hxx: Refined debug timing control:  - Added a
  SampleStatistic class (from the old deprecated libg++) library.
  - Make time statistics and printing conditionable   - Added an
  interface function to switch time stamp collection and printing
  on and off from the application (defaults to off).

2007-11-17 10:16  durk

* simgear/structure/SGSmplhist.cxx: file SGSmplhist.cxx was added
  on branch PRE_OSG_PLIB_20061029 on 2007-11-17 09:18:34 +0000

2007-11-17 10:16  durk

* simgear/structure/SGSmplhist.hxx: file SGSmplhist.hxx was added
  on branch PRE_OSG_PLIB_20061029 on 2007-11-17 09:18:34 +0000

2007-11-17 10:16  durk

* simgear/structure/SGSmplstat.cxx: file SGSmplstat.cxx was added
  on branch PRE_OSG_PLIB_20061029 on 2007-11-17 09:18:34 +0000

2007-11-17 10:16  durk

* simgear/structure/SGSmplstat.hxx: file SGSmplstat.hxx was added
  on branch PRE_OSG_PLIB_20061029 on 2007-11-17 09:18:34 +0000

2007-11-09 06:55  frohlich

* simgear/scene/model/model.cxx: Improove texture sharing.

2007-11-09 06:52  frohlich

* simgear/scene/util/SGNodeMasks.hxx: Update node masks

2007-11-05 22:42  curt

* simgear/io/: sg_binobj.cxx, sg_binobj.hxx: In the original
  flightgear native/binary scenery terrain data format, we used
  short's extensively to represent counts of objects (number of
  points, number of texture coordinates, number of traingle strips,
  etc.) and we used shorts to index into larger structures.  But
  this capped many of our structure sizes to a maximum of 32768.

  By switching to unsigned shorts in the future, we can double the
  maximum object/index counts without losing anything.	This was a
  pretty major oversight in our original specification.

  I have bumped up the native object file version from 6 to 7 and
  added code in the reader to maintain full backwards compatibilty
  with version 6 scenery files (i.e. the current 0.9.10 scenery
  release.)

  Curt.

2007-10-15 20:49  durk

* simgear/: math/SGVec2.hxx, scene/material/matlib.cxx,
  scene/sky/sphere.cxx, scene/sky/stars.cxx,
  structure/SGAtomic.hxx: - A few fixes to ensure SimGear compiles
  again on cygwin.    * on cygwin, isnan is declared in ieeepf.h
  * CYGWIN is a special case in that it is windows based, but
  sometimes	 folows unix conventions. SGAtomic compilation
  failed on an illegal	    volatile type cast without the
  additional __CYGWIN__ define check.

2007-10-15 18:20  mfranz

* simgear/scene/model/model.cxx: ignore OSG models already for
  main-models, not just submodels, so that they don't have to be
  used in <model> tags.

2007-10-15 17:51  mfranz

* simgear/scene/model/model.cxx: - warn about *.osg submodels, but
  otherwise ignore them. This allows to   use aircraft using OSG
  smoke (e.g. the Buccaneer) in fg/plib.  - warning--

2007-10-14 15:46  durk

* simgear/: screen/RenderTexture.cpp, structure/subsystem_mgr.cxx,
  structure/subsystem_mgr.hxx: * Fixed memory leak in
  RenderTexture.cpp (tiny, but still...) * Added Timestamping
  debugging code to SGSubsystems (ported from plib   branch).

2007-10-13 16:05  durk

* simgear/: environment/metar.cxx, props/props.cxx: Whoops, test
  compile before committing. Fixing a few typos here.

2007-10-13 15:51  durk

* simgear/: bucket/newbucket.cxx, environment/metar.cxx,
  io/tcp_client.cxx, props/props.cxx, screen/shader.cpp,
  screen/ssgEntityArray.cxx, structure/exception.cxx,
  structure/subsystem_mgr.cxx, structure/subsystem_mgr.hxx,
  timing/sg_time.cxx: * Remove inspection of snprintf return value
  when it's obvious that printed   values can never be longer than
  the buffer size (leading to better   readable code). Otherwise,
  make sure to check that return value is not >=   to the max
  buffer size.	* Integrate "time elapsed" logging mechanism into
  SGSubsystem base class   to allow for more flexible tracking of
  timing errors.

2007-10-13 10:16  durk

* simgear/: bucket/newbucket.cxx, environment/metar.cxx,
  io/tcp_client.cxx, props/props.cxx, screen/RenderTexture.cpp,
  screen/shader.cpp, screen/ssgEntityArray.cxx,
  structure/exception.cxx, timing/sg_time.cxx, timing/sg_time.hxx:
  Error checking: * Replace sprintf by snprintf and check for
  overflows.  * Fix a tiny memory leak RenderTexture, as reported
  by valgrind.

2007-10-13 00:46  timoore

* simgear/scene/model/model.cxx: Add ability to override existing
  model files with .osg files.

  This enables off-line optimization of models and other special
  effects.

2007-10-13 00:46  timoore

* simgear/scene/model/model.cxx: Manage OSG object cache explicitly

  Don't have Registry::ReadNodeImplementation store its results in
  the object cache; instead store the optimized model in the cache.

2007-10-10 14:45  mfranz

* simgear/props/: props_io.cxx: better standard compliance: allow
  empty top level tags (<PropertyList>)

2007-10-04 22:53  timoore

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Don't reset the random
  texture base when rebuilding a cloud layer This fix removes
  obnoxious visuals (texture jumping) when a cloud layer is moved
  due to a metar update or, more significantly, when switching from
  metar to a scenario. Also, I switched to using a TexMat to
  displace the cloud texture in order to avoid writing the texture
  array every frame.

2007-10-02 23:43  timoore

* simgear/scene/sky/cloud.cxx: Order the cloud layers properly
  using OSG RenderBin

2007-09-30 21:51  timoore

* simgear/scene/tgdb/SGOceanTile.cxx: ocean state set in
  osg::Geometry

  Put the ocean tile state set in osg::Geometry, not the
  osg::Geode, so that is readily available during intersection
  testing and can be used to find the corresponding SGMaterial.

  Problem reported by grtux (gh.robin@laposte.net)

2007-09-30 13:01  durk

* simgear/structure/: subsystem_mgr.cxx, subsystem_mgr.hxx: Add
  timing debugging code.

2007-09-23 13:37  durk

* simgear/structure/: subsystem_mgr.cxx, subsystem_mgr.hxx:
  Modified version of Frederic Bouvier's subsystem timing profiler.

2007-09-06 00:12  timoore

* simgear/scene/model/SGMaterialAnimation.cxx: Fix material
  animations with only a <texture-prop>

  On 8/31/07, K. Hoercher <wbhoer@gmail.com> wrote:
  > > Some notes:
  > > - I found that in order to make the example from
  model-howto.html work
  > > ( starting at "To make a texture replaceable at runtime") one
  has to
  > > specify a valid (i.e. loadable) <texture>  in the material
  animation.

  The cause seems to be the condition in SGMaterialAnimation.cxx
  l.277 ignoring any texture update by the UpdateCallback (only
  there <texture-prop> is looked at)  without an already existing
  stateSet.  That in turn will not be created with a <texture-prop>
  alone l. 379ff.

  Unless I overlooked some compelling reason contradicting, I'd
  like to suggest allowing for a stateSet to be created for those
  situations too. I think that would match the behaviour of
  animation.cxx (PRE_OSG_PLIB_20061029) and is imho the more
  expected and also documented one.

2007-09-01 14:57  durk

* configure.ac: Change "FlightGear" to "SimGear" of course.

2007-09-01 14:55  durk

* Makefile.am, README.OSG, configure.ac: Added a check for
  OpenSceneGraph and created some basic documentation.

2007-09-01 12:04  durk

* configure.ac: Add separate check for openal and ALUT, as these
  are now separate.

2007-09-01 11:43  durk

* Makefile.am, README.plib, README.OpenAL: Documentation fixes:
  Updated the contents of README.plib and README.OpenAL Also make
  sure that these files are included in the release. This is
  required, since ./configure refers to these files.

2007-08-12 23:03  timoore

* simgear/: misc/texcoord.cxx, misc/texcoord.hxx,
  scene/tgdb/SGOceanTile.cxx, scene/util/Makefile.am,
  scene/util/VectorArrayAdapter.hxx: Add curvature to ocean tiles.

  An apron around the tile hides cracks with coastal tiles.

  The VectorArrayAdapter, which lives in the simgear namespace, is
  a useful utility class for treating vectors as 2D arrays.

2007-08-12 23:02  timoore

* simgear/scene/tgdb/SGOceanTile.cxx: Ocean tile detail work in
  progress

2007-08-12 15:40  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2007-08-12 15:32  fredb

* simgear/scene/model/SGTranslateTransform.cxx: Fix a typo

2007-08-07 07:26  frohlich

* projects/VC7.1/SimGear.vcproj, simgear/math/Makefile.am,
  simgear/math/SGGeoc.hxx, simgear/math/SGGeodesy.cxx,
  simgear/math/SGGeodesy.hxx, simgear/math/polar3d.cxx,
  simgear/math/polar3d.hxx, simgear/math/sg_geodesy.cxx,
  simgear/math/sg_geodesy.hxx, simgear/math/sg_memory.h,
  simgear/math/sg_types.hxx: Modified Files:
  projects/VC7.1/SimGear.vcproj projects/VC8/SimGear.vcproj
  simgear/math/Makefile.am simgear/math/SGGeoc.hxx
  simgear/math/SGGeodesy.cxx simgear/math/SGGeodesy.hxx
  simgear/math/polar3d.hxx simgear/math/sg_geodesy.hxx
  simgear/math/sg_types.hxx Removed Files:
  simgear/math/polar3d.cxx simgear/math/sg_geodesy.cxx
  simgear/math/sg_memory.h:	  Remove sg_memory.h It is unused
  anyway and should not be required	  in a c++ world. Move
  distance course functions to the SG* type  system. Move the
  implementation into SGGeodesy.cxx. Remove some of	 the old
  Point3D Based sg* functions that are already unused.

2007-08-02 00:46  andy

* simgear/nasal/: lib.c: Melchior discovered that call(call) would
  crash due to a longstanding bug with argument parsing

2007-08-02 00:33  andy

* simgear/nasal/: codegen.c: Fix crash in code generator for some
  bad lvalue expressions

2007-07-31 22:57  andy

* simgear/nasal/: naref.h: Add ppc64 to the list of supported
  platforms based on testing by Tom Callaway at Red Hat

2007-07-31 08:39  timoore

* simgear/scene/tgdb/SGOceanTile.cxx: Fix ocean texture scaling
  problem.

2007-07-31 03:21  curt

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx: Remove an
  extern SGSky *thesky reference that isn't used in the code
  anyway.  The original code forced a dependency in SimGear back to
  something that is defined globaly in FlightGear, not a great
  strategy for a library with some "general purpose" intentions.

2007-07-31 03:19  curt

* simgear/scene/sky/: cloud.cxx, cloud.hxx, cloudfield.cxx,
  cloudfield.hxx, sky.cxx: cloudfield had an extern SGSky *thesky
  reference that is defined in FlightGear.  It is really odd that
  the code is structured this way, so I did some minor
  modifications to the API to pass this data down in a more well
  defined way.

2007-07-30 00:32  timoore

* simgear/scene/: model/model.cxx, tgdb/Makefile.am,
  tgdb/SGReaderWriterBTG.cxx, tgdb/SGReaderWriterBTG.hxx,
  tgdb/SGReaderWriterBTGOptions.hxx, tgdb/userdata.cxx: OSG Reader
  and Writer for BTG files

  This is part of a somewhat long road towards terrain database
  paging using OSG's database pager thread.

2007-07-27 21:27  frohlich

* simgear/math/SGGeoc.hxx: Modified Files:
  simgear/math/SGGeoc.hxx: Fix unit conversion problem

2007-07-24 00:00  timoore

* Thanks: SimGear: Typo in Thanks file

  Fix typo

  Author: Hans Ulrich Niedermann <hun@n-dimensional.de> Committer:
  Tim Moore <moore@redhat.com>

2007-07-23 23:45  timoore

* configure.ac: SimGear: Properly print compilers in configure.ac

  Print $CC and $CXX using the proper configure.ac mechanism

  Author: Hans Ulrich Niedermann <hun@n-dimensional.de>
  Signed-off-by: Tim Moore <timoore@redhat.com>

2007-07-23 00:37  timoore

* simgear/: bucket/Makefile.am, io/Makefile.am, magvar/Makefile.am,
  misc/Makefile.am, props/Makefile.am, route/Makefile.am,
  screen/Makefile.am, serial/Makefile.am, sound/Makefile.am: Allow
  parallel make ("make -j2") by giving make the opportunity to
  determine dependencies.

  As in subdirectory foo/bar, $(top_builddir)/foo/bar is defined to
  be the current directory, this does not cause any regressions.

  From Hans Ulrich Niedermann (hun@n-dimensional.de)

2007-07-22 22:06  timoore

* simgear/scene/model/: SGOffsetTransform.cxx,
  SGOffsetTransform.hxx, SGRotateTransform.cxx,
  SGRotateTransform.hxx, SGScaleTransform.cxx,
  SGScaleTransform.hxx, SGTranslateTransform.cxx,
  SGTranslateTransform.hxx, placementtrans.cxx, placementtrans.hxx:
  Support for reading and writing nodes in .osg files, plus some
  new accessors.

2007-07-22 15:58  mfranz

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx: coding style fixes

2007-07-22 15:50  mfranz

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx, xmlsound.cxx: Maik
  JUSTUS: workaround for broken Doppler effect in OpenAL

  mf: this patch is meant to be removed as soon as OpenAL got
  fixed. (The	  OpenAL developers acknowleged the bug and
  announced that it'll get     fixed.) For removal try	   $ cd
  simgear/sound     $ cvs diff -rAFTER_OPENAL_DOPPLER_WORKAROUND
  -rBEFORE_OPENAL_DOPPLER_WORKAROUND|patch

2007-07-22 15:33  mfranz

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, xmlsound.cxx: Maik JUSTUS: Doppler fixes
  (add option to turn off Doppler for sounds that
  shouldn't be affected -- marker beep, ATIS messages, etc.)

  mf: this is the first part of the original patch. It is supposed
  to contain	 fixes that are not caused by OpenAL bugs, and thus
  aren't meant to be	 reverted later. The second part will
  contain a temprary workaround for	OpenAL bugs. Unfortunately,
  I had to do the split myself as the contributor     refused to do
  it.

2007-07-17 16:52  mfranz

* simgear/props/: props.cxx: - close loophole through which one
  could sneak in illegal property names   containing slashes,
  colons and all sorts of evil characters. In Nasal   this could be
  done via props.globals.getChild("1!@#$//[]{}", 0, 1).setValue(0);
  The cause is that getChild() hands the given name directly over
  to an   alternative SGPropertyNode ("convenience") constructor
  which sets the   name without any checks.  - unify exception
  messages: first character is lower case

2007-07-12 12:43  mfranz

* simgear/scene/model/shadanim.cxx: allocate W*H*4 bytes (rather
  than W*H*3), as it turned out that glGetTexImage() returns that
  much for the default red/white chequer texture, which is used
  when the requested texture couldn't be found.  This could be a
  bug in the implementation (nvidia/100.14.11/Linux x86) or be
  normal for compressed textures
  (GL_COMPRESSED_RGBA_S3TC_DXT5_EXT).  The OSG developers aren't
  sure about their sizes either (see comment in src/osg/Image.cpp,
  revision 7067: computePixelSizeInBits()).

  for the 2x2 RGB red/white chequer texture:

  0 1 2 3 4 5 6 7 8 9 a b c d e f
    IN: ff0000ffffffffffffff0000	  (see
  src/ssg/ssgLoadTexture)
    OUT: f70000ffffff....fffffff70000....

  where the positions marked with a dot remained untouched.

2007-07-12 01:39  mfranz

* simgear/scene/model/: model.cxx, model.hxx: reset texture path
  after loading sub-models, which may have changed it

2007-07-08 10:43  frohlich

* simgear/route/route.hxx: Modified Files:
  simgear/route/route.hxx: Remove unused include.

2007-07-08 10:43  frohlich

* simgear/route/: waypoint.cxx, waypoint.hxx: Modified Files:
  simgear/route/waypoint.hxx simgear/route/waypoint.cxx: Use const
  refs where possible.

2007-07-07 20:50  mfranz

* simgear/sound/: sample_openal.cxx, xmlsound.cxx: change sign &
  initialize direction (this was in a later patch that I had
  missed)

2007-07-07 20:37  mfranz

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx, xmlsound.cxx: Maik
  JUSTUS:

  "[...] switches of the Doppler calculation of OpenAL and adds a
  own Doppler calculation."

  mf: necessary because OpenAL Doppler on Windows is broken, which
  the	  openal developers acknowledge. The source revisions
  before and after     the patch was applied are tagged with
  BEFORE_OPENAL_WORKAROUND and	   AFTER_OPENAL_WORKAROUND, so that
  one can easier find and remove     the changes if required
  (which is quite unlikely. :-)

2007-07-02 22:06  mfranz

* simgear/ephemeris/stardata.cxx: nope, "return false" is better
  here, sorry

2007-07-02 22:02  mfranz

* simgear/ephemeris/stardata.cxx: s/exit(.*)/throw/  .. again
  leaving the SG_LOG in until I know why exceptions dont' work
  through static sg libs

2007-07-02 21:57  mfranz

* simgear/scene/tgdb/leaf.cxx: replace exit() by throw. Leaving the
  SG_LOG message in, because I observed unexpected problems with
  throwing exceptions from sg.	:-/

2007-07-02 17:42  mfranz

* simgear/timing/: lowleveltime.cxx, timezone.cxx: replace exit()
  by throw sg_exception(). Of course, we have to be aware that
  interdependencies between sg libs are generally unwelcome, but
  sg_exception is a rather basic part, and it's already used by
  xml, props, scene, sound and, of course, structure. Since props
  and xml are core libs, we can assume that sg_exceptions are
  available.  (OK'ed by Curt)

2007-07-02 14:55  mfranz

* simgear/debug/logstream.hxx: add SG_ORIGIN macro that expands to
  a string  __FILE__":"__LINE__ Note that __LINE__ is a number and
  can't be directly used in string context, which makes the macro
  worthwhile.  (IMHO :-)

2007-07-02 14:55  mfranz

* simgear/debug/logstream.hxx: add SG_ORIGIN macro that expands to
  a string  __FILE__":"__LINE__ Note that __LINE__ is a number and
  can't be directly used in string context, which makes the macro
  worthwhile. (IMHO :-)

2007-06-30 00:45  mfranz

* simgear/environment/: visual_enviro.cxx, visual_enviro.hxx: back
  out last changes (radar patch)

2007-06-29 12:46  mfranz

* simgear/xml/: easyxml.cxx, testEasyXML.cxx: easyxml.cxx: add
  missing endXML visitor call testEasyXML.cxx: beef it up

2007-06-25 17:29  mfranz

* simgear/environment/: visual_enviro.cxx, visual_enviro.hxx:
  revert last change (part of the radar patch). It changed the
  interface for no good reason and didn't make that much sense for
  the general case.  (It had added a flag with the meaning
  "this-cloud-is-an-aircraft". sg isn't only used for fgfs. :-)

2007-06-24 10:09  mfranz

* simgear/environment/: visual_enviro.cxx, visual_enviro.hxx:
  Vivian MEAZZA: add support for aircraft radar signatures

2007-06-23 23:55  mfranz

* simgear/environment/: visual_enviro.cxx, visual_enviro.hxx:
  Vivian MEAZZA: add aircraft signature support to weather radar

2007-06-23 18:48  mfranz

* simgear/sound/: xmlsound.cxx: don't only complain that the volume
  is larger than 1.0, but say how much it actually is

2007-06-21 23:46  mfranz

* simgear/sound/: sample_openal.cxx, xmlsound.cxx: Maik JUSTUS:
  fix/implement directional sound

2007-06-19 20:22  mfranz

* simgear/props/: props.cxx: d'oh ... beautify the TRACE message
  that we actually see!  :-}

2007-06-19 20:10  mfranz

* simgear/props/: props.cxx: beautify TRACE message

2007-06-17 23:01  mfranz

* simgear/scene/model/: animation.cxx: sooner than planned: fix
  "scale" animation offsets (1 -> 0)

2007-06-16 18:14  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2007-06-16 18:13  fredb

* simgear/scene/tgdb/SGDirectionalLightBin.hxx: MSVC 7 compilation

2007-06-14 22:20  mfranz

* simgear/screen/: screen-dump.cxx: Nick WARNE: add file name to
  screenshot info line

2007-06-11 18:09  mfranz

* simgear/props/: props.cxx: advance tracing messages from SG_INFO
  to SG_ALERT. If a developer has demanded tracing, then he
  shouldn't get these important messages buried in thousands of
  lines of meaningless bulk.

2007-06-09 20:36  mfranz

* simgear/environment/: metar.cxx: - allow for (rather unusual)
  ////// cloud groups - fix potential use of uninitialized memory:
  dew

2007-06-08 08:50  frohlich

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  util/SGSceneFeatures.cxx, util/SGSceneFeatures.hxx: Modified
  Files:       simgear/scene/material/mat.cxx
  simgear/scene/material/mat.hxx
  simgear/scene/util/SGSceneFeatures.cxx
  simgear/scene/util/SGSceneFeatures.hxx:	  Olaf Flebbe: Make
  use of SGSceneFeatues for anisotropic filtering,	clean up.

2007-06-08 08:40  frohlich

* simgear/scene/tgdb/leaf.cxx: Removed Files:
  simgear/scene/tgdb/leaf.cxx: Now obsolete but not yet removed.

2007-06-03 20:28  frohlich

* simgear/scene/tgdb/SGOceanTile.cxx: Modified Files:
  scene/tgdb/SGOceanTile.cxx: add missing transform for the ocean
  tile.

2007-06-03 20:21  frohlich

* simgear/scene/: model/model.cxx, util/SGSceneFeatures.cxx,
  util/SGSceneFeatures.hxx: Modified Files:
  simgear/scene/model/model.cxx
  simgear/scene/util/SGSceneFeatures.cxx
  simgear/scene/util/SGSceneFeatures.hxx:	  Make sure
  textures are shared. Do not rely on a graphics	context to
  be available on model loading.

2007-05-31 00:49  andy

* simgear/nasal/: code.c, code.h, data.h, lib.c, misc.c, nasal.h:
  Sync with Nasal upstream.  Mostly fixes to naContinue(), which
  FlightGear doesn't use.  Also includes a performance fix for the
  call() builtin that should help Melchior, who was measuring lower
  performance for the props.Node() interface than the
  getprop/setprop API.

2007-05-30 15:07  frohlich

* simgear/scene/model/SGMaterialAnimation.cxx: Modified Files:
  simgear/scene/model/SGMaterialAnimation.cxx:	  Olaf Flebbe: Use
  brakets around bitwise operations.	 Greetings from LinuxTag,
  Berlin ... :)

2007-05-30 14:34  curt

* simgear/bucket/newbucket.hxx: I guess we aren't using explicit
  destructors here.

2007-05-29 21:38  curt

* simgear/bucket/newbucket.hxx: Make an explicit destructor so the
  compiler doesn't get confused about non matching exception types
  with the implicitely defined destructor.

2007-05-28 09:13  frohlich

* simgear/scene/model/: Makefile.am, SGRotateTransform.cxx,
  SGRotateTransform.hxx, SGScaleTransform.cxx,
  SGScaleTransform.hxx, SGTranslateTransform.cxx,
  SGTranslateTransform.hxx, animation.cxx, animation.hxx: Modified
  Files:	simgear/scene/model/Makefile.am
  simgear/scene/model/animation.cxx
  simgear/scene/model/animation.hxx Added Files:
  simgear/scene/model/SGRotateTransform.cxx
  simgear/scene/model/SGRotateTransform.hxx
  simgear/scene/model/SGScaleTransform.cxx
  simgear/scene/model/SGScaleTransform.hxx
  simgear/scene/model/SGTranslateTransform.cxx
  simgear/scene/model/SGTranslateTransform.hxx:   Factor out some
  useful classes.

2007-05-28 07:13  frohlich

* simgear/scene/material/: mat.cxx, mat.hxx: Modified Files:
  simgear/scene/material/mat.cxx simgear/scene/material/mat.hxx:
  Olaf Flebbe: Improoved texture filtering.

2007-05-28 07:06  frohlich

* simgear/structure/SGAtomic.hxx: Modified Files:
  SGAtomic.hxx: Also use atomic compiler intrinsics on SGI

2007-05-28 07:00  frohlich

* simgear/: bucket/newbucket.cxx, bucket/newbucket.hxx,
  io/decode_binobj.cxx, io/sg_binobj.cxx, io/sg_binobj.hxx,
  math/SGVec2.hxx, math/SGVec3.hxx, math/SGVec4.hxx,
  scene/material/mat.hxx, scene/material/matlib.cxx,
  scene/material/matlib.hxx, scene/model/Makefile.am,
  scene/model/SGOffsetTransform.cxx,
  scene/model/SGOffsetTransform.hxx, scene/tgdb/Makefile.am,
  scene/tgdb/SGDirectionalLightBin.hxx, scene/tgdb/SGLightBin.hxx,
  scene/tgdb/SGOceanTile.cxx, scene/tgdb/SGOceanTile.hxx,
  scene/tgdb/SGTexturedTriangleBin.hxx,
  scene/tgdb/SGTriangleBin.hxx, scene/tgdb/SGVasiDrawable.cxx,
  scene/tgdb/SGVasiDrawable.hxx, scene/tgdb/SGVertexArrayBin.hxx,
  scene/tgdb/leaf.hxx, scene/tgdb/obj.cxx, scene/tgdb/obj.hxx,
  scene/tgdb/pt_lights.cxx, scene/tgdb/pt_lights.hxx,
  scene/tgdb/vasi.hxx, scene/util/Makefile.am,
  scene/util/SGEnlargeBoundingBox.cxx,
  scene/util/SGEnlargeBoundingBox.hxx, scene/util/SGNodeMasks.hxx,
  scene/util/SGSceneFeatures.cxx, scene/util/SGSceneFeatures.hxx,
  scene/util/SGTextureStateAttributeVisitor.cxx: Modified Files:
  simgear/bucket/newbucket.cxx simgear/bucket/newbucket.hxx
  simgear/io/decode_binobj.cxx simgear/io/sg_binobj.cxx
  simgear/io/sg_binobj.hxx simgear/math/SGVec2.hxx
  simgear/math/SGVec3.hxx simgear/math/SGVec4.hxx
  simgear/scene/material/mat.hxx  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx
  simgear/scene/model/Makefile.am simgear/scene/tgdb/Makefile.am
  simgear/scene/tgdb/obj.cxx simgear/scene/tgdb/obj.hxx
  simgear/scene/tgdb/pt_lights.cxx
  simgear/scene/tgdb/pt_lights.hxx
  simgear/scene/util/Makefile.am
  simgear/scene/util/SGNodeMasks.hxx
  simgear/scene/util/SGTextureStateAttributeVisitor.cxx Added
  Files:      simgear/scene/model/SGOffsetTransform.cxx
  simgear/scene/model/SGOffsetTransform.hxx
  simgear/scene/tgdb/SGDirectionalLightBin.hxx
  simgear/scene/tgdb/SGLightBin.hxx
  simgear/scene/tgdb/SGOceanTile.cxx
  simgear/scene/tgdb/SGOceanTile.hxx
  simgear/scene/tgdb/SGTexturedTriangleBin.hxx
  simgear/scene/tgdb/SGTriangleBin.hxx
  simgear/scene/tgdb/SGVasiDrawable.cxx
  simgear/scene/tgdb/SGVasiDrawable.hxx
  simgear/scene/tgdb/SGVertexArrayBin.hxx
  simgear/scene/util/SGEnlargeBoundingBox.cxx
  simgear/scene/util/SGEnlargeBoundingBox.hxx
  simgear/scene/util/SGSceneFeatures.cxx
  simgear/scene/util/SGSceneFeatures.hxx Removed Files:
  simgear/scene/tgdb/leaf.hxx simgear/scene/tgdb/vasi.hxx:
  Reorganize tile loaders.	  Build bigger leafs for the tiles.
  Move runway light colors into materials.xml.	Split out
  classes that might be useful at other places. 	Avoid
  static storage on binobject loading.

2007-05-25 17:49  andy

* simgear/nasal/naref.h: Sync with trunk

2007-05-25 17:49  andy

* simgear/nasal/naref.h: GCC on ppc linux uses a different
  architecture symbol than the same compiler on OS X

2007-05-18 09:29  frohlich

* simgear/math/SGVec3.hxx: Modified Files:	SGVec3.hxx: Fix a
  problem in perpendicular triangle computation.	Solves
  problem with invalid triangles in ground picking ...

2007-05-18 06:46  frohlich

* simgear/math/: SGVec2.hxx, SGVec3.hxx, SGVec4.hxx, point3d.hxx:
  Modified Files:	SGVec2.hxx SGVec3.hxx SGVec4.hxx
  point3d.hxx: Provide ordering  relations for use with std::less
  in tree bases std:: containers.

2007-05-16 18:08  curt

* simgear/sound/: openal_test1.cxx: Fix a compiler warning.

2007-05-16 18:06  curt

* simgear/screen/: RenderTexture.cpp: Fix various compiler
  warnings.

2007-05-16 00:28  mfranz

* Thanks: Tim Moore

2007-05-16 00:02  mfranz

* Thanks: add Mathias and Maik (to make it look less selfish that I
  add myself :-) I'm sure I forgot a lot of people, but it's a
  start.

2007-05-14 19:55  curt

* Doxyfile, Makefile.am, NEWS, SimGear.dsp, configure.ac: Updates
  for impending plib release.

2007-05-13 13:52  mfranz

* simgear/scene/material/: mat.cxx, mat.hxx:
  s/resistence/resistance/

2007-05-08 08:11  frohlich

* simgear/: bucket/newbucket.hxx, scene/material/mat.cxx,
  scene/material/matlib.cxx, scene/material/matlib.hxx,
  scene/model/location.cxx, scene/tgdb/apt_signs.cxx,
  scene/tgdb/leaf.cxx, scene/tgdb/leaf.hxx, scene/tgdb/obj.cxx,
  scene/tgdb/obj.hxx, scene/util/SGUpdateVisitor.hxx: Modified
  Files:   simgear/bucket/newbucket.hxx
  simgear/scene/material/mat.cxx
  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx
  simgear/scene/model/location.cxx
  simgear/scene/tgdb/apt_signs.cxx simgear/scene/tgdb/leaf.cxx
  simgear/scene/tgdb/leaf.hxx simgear/scene/tgdb/obj.cxx
  simgear/scene/tgdb/obj.hxx
  simgear/scene/util/SGUpdateVisitor.hxx: Reorganize scenegraph to
  simplify top level structure.

2007-05-07 16:03  mfranz

* simgear/props/: props.hxx: Add method to return the number of
  attached listeners. Listeners have become a much more important
  feature than they were two years or something ago, and it's
  helpful for debugging and exploration to get this important node
  property shown in property tree dumps or in the property browser
  (verbose mode).

2007-05-06 19:33  mfranz

* simgear/props/: props_io.cxx: - fix bug where a property tree
  saved with writeProperties() and read back   in with
  readProperties() would not look the same, because element indices
    of '0' were even dropped when a node has a "secret" value *and*
  children

  - introduce "omit-node" modifier attribute for the "include"
  attribute.	This inserts the given file in place of the
  including node, while the    node is dropped. This is desirable
  for multiple includes (which can't	be done by multiply using
  the "include" attribute, as this isn't valid	  XML spec syntax)

2007-05-05 13:16  mfranz

* simgear/scene/model/SGMaterialAnimation.cxx: better warning text
  for <global> in material animations

2007-05-05 11:15  frohlich

* simgear/scene/model/: SGMaterialAnimation.cxx,
  SGMaterialAnimation.hxx: Modified Files:
  SGMaterialAnimation.cxx SGMaterialAnimation.hxx:	  Tim
  Moore: overhaul the material animation.

2007-05-03 21:46  frohlich

* simgear/scene/model/: animation.hxx, shadanim.cxx: Modified
  Files:    simgear/scene/model/animation.hxx
  simgear/scene/model/shadanim.cxx: Tim Moore: the crom shader.

2007-05-03 00:34  mfranz

* simgear/nasal/lib.c: Andy ROSS: "Fix crash when sorting
  newly-allocated empty vectors" (merge from HEAD)

2007-05-03 00:29  andy

* simgear/nasal/lib.c: Empty vectors work much better as the result
  of sorting an empty array than nil does...

2007-05-03 00:24  andy

* simgear/nasal/lib.c: Fix crash when sorting newly-allocated empty
  vectors

2007-04-29 01:13  mfranz

* simgear/screen/: texture.cxx: don't rely on a compressed scanline
  being properly closed (GIMP apparently generates corrupted files)

2007-04-28 14:30  mfranz

* simgear/screen/: texture.cxx: SGTexture::read_rgb(a)_texture: -
  support greyscale and greyscale/alpha format - cleanup & make it
  faster

2007-04-21 14:13  frohlich

* simgear/scene/material/: mat.cxx, mat.hxx: Modified Files:
  simgear/scene/material/mat.cxx simgear/scene/material/mat.hxx:
  Olaf Flebbe: make anisotroüpic filtering configurable.

2007-04-06 22:35  andy

* simgear/nasal/codegen.c: sync with trunk

2007-04-06 22:35  andy

* simgear/nasal/codegen.c: Melchior found a bug with negative
  values in default function arguments

2007-04-06 16:52  andy

* simgear/nasal/: lib.c: sync with Nasal CVS (added a sort()
  builtin)

2007-04-06 11:54  mfranz

* simgear/route/: route.cxx, route.hxx, routetest.cxx: Csaba
  HALASZ: - fix bug that messed up leg distances after inserting
  and deleting waypoints   not at the end of the route - move
  add_waypoint() and delete_waypoint from hxx to cxx - beef up
  routetest

2007-04-03 13:35  fredb

* projects/VC7.1/SimGear.vcproj: Update MSVC 7.1 projects

2007-04-03 13:25  fredb

* simgear/scene/model/model.cxx: Avoid potential memory leak
  problems when exceptions are thrown by using reference objects

2007-04-02 23:32  andy

* simgear/nasal/naref.h: sync with trunk

2007-04-02 23:32  andy

* simgear/nasal/naref.h: Fix typo in _M_IX86

2007-04-02 20:28  andy

* simgear/nasal/lib.c: sync with trunk

2007-04-02 20:28  andy

* simgear/nasal/lib.c: Rewrite substr() to properly clamp its
  argument ranges and handle negative start arguments as
  offset-from-end values

2007-04-02 19:34  andy

* simgear/nasal/thread-win32.c: sync with trunk

2007-04-02 19:34  andy

* simgear/nasal/thread-win32.c: Add missing free functions for
  win32

2007-04-02 18:15  andy

* simgear/nasal/: lib.c, mathlib.c: sync with trunk

2007-04-02 18:14  andy

* simgear/nasal/: lib.c, mathlib.c: Use __FUNCTION__, which works
  on gcc and MSVC 7/8, instead of __func__, which while
  standardized works only with gcc.  I'll wait for bug reports from
  VC6 before bothering with fallback code...

2007-03-31 14:23  mfranz

* simgear/math/: interpolater.cxx, interpolater.hxx: merge from
  HEAD (required for John DENKER's altimeter changes):

  Mathias FROEHLICH: - lookup by using a std::map.  - Enable
  reading tables directly from our dom like tree.

2007-03-30 18:59  andy

* simgear/nasal/mathlib.c: sync with trunk

2007-03-30 18:42  andy

* simgear/nasal/mathlib.c: Melchior points out that NaN/Inf
  behavior is not platform-independent.  So toss a runtime error
  ("floating point error") when any of the math library functions
  produce a non-finite value.  Note that these are not the only
  locations that can do that (simply dividing by zero will produce
  an Inf), but it's still proper behavior.

2007-03-29 20:52  andy

* simgear/nasal/: Makefile.am, bitslib.c, code.c, code.h,
  codegen.c, data.h, gc.c, hash.c, iolib.c, lex.c, lib.c,
  mathlib.c, misc.c, naref.h, nasal.h, parse.c, parse.h, string.c,
  thread-posix.c, thread-win32.c, threadlib.c, utf8lib.c, vector.c:
  sync with trunk

2007-03-29 20:50  andy

* simgear/nasal/: Makefile.am, bitslib.c, code.c, code.h,
  codegen.c, data.h, gc.c, hash.c, iolib.c, lex.c, lib.c,
  mathlib.c, misc.c, naref.h, nasal.h, parse.c, parse.h, string.c,
  thread-posix.c, thread-win32.c, threadlib.c, utf8lib.c, vector.c:
  Sync with Nasal CVS (soon to become Nasal 1.1).  Notable new
  features:

  Nasal now supports calls to "subcontexts" and errors can be
  thrown across them, leading to complete stack traces when call()
  is used, instead of the truncated ones we now see.

  Vectors can now be concatenated using the ~ operator that used to
  work only for strings.

  Better runtime error messages in general due to a fancier
  naRuntimeError() implementation

  A big data size shrink on 64 bit systems; the size of a naRef
  dropped by a factor of two.

  "Braceless code blocks" have been added to the parser, so you can
  write expressions like "if(a) b();" just like in C.  Note that
  there's still a parser bug in there that fails when you nest a
  braced block within a braceless one.

  Character constants that appear in Nasal source code can now be
  literal multibyte UTF8 characters (this was always supported for
  string literals, but character constants were forced to be a
  single byte).

  New modules: "bits", "thread", "utf8" and (gulp...) "io".  The
  bits library might be useful to FlightGear, the utf8 one probably
  not as Plib does not support wide character text rendering.  The
  thread library will work fine for spawning threads to do Nasal
  stuff, but obviously contact with the rest of FlightGear must be
  hand-synchronized as FlightGear isn't threadsafe.  The io library
  is no doubt the most useful, as it exposes all the basic stdio.h
  facilities; it's also frighteningly dangerous when combined with
  networked code...

2007-03-29 20:50  andy

* simgear/nasal/naref.h: file naref.h was added on branch
  PRE_OSG_PLIB_20061029 on 2007-03-29 18:52:33 +0000

2007-03-29 20:50  andy

* simgear/nasal/threadlib.c: file threadlib.c was added on branch
  PRE_OSG_PLIB_20061029 on 2007-03-29 18:52:34 +0000

2007-03-29 20:50  andy

* simgear/nasal/utf8lib.c: file utf8lib.c was added on branch
  PRE_OSG_PLIB_20061029 on 2007-03-29 18:52:34 +0000

2007-02-17 11:50  mfranz

* simgear/props/: props.cxx, props.hxx: Csaba HALASZ: when a
  path_cache ceases to exist, unregister from all nodes
    that had been told that this node is linking to them

  mf: remove old erase-by-key methods; they are now unused and
  always were	  problematic, so they won't be used in the future
  either

2007-02-16 16:32  mfranz

* simgear/props/: props.cxx, props.hxx: - don't leak node in both
  hash_table::bucket::erase() - remove bad code from
  hash_table::bucket::erase(const char *) that was   introduced
  with the last patch. (This function isn't used anywhere and	is
  scheduled for removal. Leaving it in for now as a reference.) -
  remove leaves first in remove_from_path_caches()

  - cosmetics: indentation, one trailing space, variable name
  change, comment   (Sorrry for mixing that with actual code, but I
  think it's easy to see.)

2007-02-15 00:14  andy

* simgear/structure/SGBinding.cxx: Don't crash when destroying a
  SGBinding object if the property node it was loaded from lacks a
  parent.  Patch from ndim on IRC

2007-02-11 12:05  mfranz

* simgear/props/: props.cxx, props.hxx: Maik JUSTUS:

  """ - make every node maintain list of properties that link to it
  - add functions to erase node by address from hash bucket/entry
  in their   path caches, so that all references can be removed -
  if a node is removed, it (and all children, grandchildren, ...)
  calls   all linked properties to remove them from their
  path-cache

  This fixes problems with the aerotow over multiplayer and maybe
  some other problems, where nodes are queried by name.  """

2007-02-07 20:21  andy

* simgear/scene/model/animation.cxx: "bias" argument to texture
  animations by Ron Jensen

2007-02-05 22:41  mfranz

* simgear/io/: sg_binobj.cxx: fix error message

2007-02-02 19:16  frohlich

* simgear/ephemeris/: ephemeris.cxx, ephemeris.hxx, stardata.cxx:
  Modified Files:	ephemeris.cxx ephemeris.hxx stardata.cxx:
  one must not do changes just before checkin,    one most not
  do changes just before checkin,    [ last message repeated 100
  times ]

2007-02-02 19:09  frohlich

* simgear/ephemeris/: ephemeris.cxx, ephemeris.hxx, stardata.cxx,
  stardata.hxx: Modified Files: 	ephemeris.cxx ephemeris.hxx
  stardata.cxx stardata.hxx: Throw out sg.h

2007-02-02 08:00  frohlich

* simgear/scene/: model/animation.cxx, util/SGNodeMasks.hxx:
  Modified Files:    simgear/scene/util/SGNodeMasks.hxx
  simgear/scene/model/animation.cxx:	  More finegrained cull
  masks

2007-01-30 21:12  frohlich

* simgear/math/: SGIntersect.hxx, SGVec3.hxx, SGVec4.hxx: Modified
  Files:       SGIntersect.hxx SGVec3.hxx SGVec4.hxx: Add
  convinience methods

2007-01-29 09:19  fredb

* simgear/scene/model/shadanim.cxx: restore 'double checked
  locking'

2007-01-28 21:04  frohlich

* simgear/scene/model/model.cxx: Modified Files:	model.cxx:
  Better texture sharing, fix problem with rotation order

2007-01-28 21:03  frohlich

* simgear/structure/SGAtomic.cxx: Modified Files:
  simgear/structure/SGAtomic.cxx: Plug memory leak originating from
  wrong atomic fallback operations.

2007-01-26 21:30  fredb

* simgear/scene/model/shadanim.cxx: Ensure a reference on the cube
  map texture is always held

2007-01-26 09:02  fredb

* simgear/scene/model/: animation.cxx, animation.hxx, shadanim.cxx:
  Fix a memory leak and behave more nicely on shared pointers

2007-01-23 11:07  ehofman

* simgear/sound/: Makefile.am, openal_test1.cxx, openal_test2.cxx:
  [no log message]

2007-01-21 12:15  fredb

* simgear/scene/model/animation.cxx: Better fix for the constant
  scale factor problem

2007-01-21 11:33  fredb

* simgear/scene/model/animation.cxx: Support constant scaling
  factor

2007-01-17 18:12  mfranz

* simgear/misc/sg_path.cxx: Frederic BOUVIER: "Don't segfault when
  dir is empty"

2007-01-16 22:34  fredb

* simgear/misc/sg_path.cxx: Don't segfault when dir is empty

2007-01-15 20:01  frohlich

* simgear/scene/model/animation.cxx: Modified Files:
  animation.cxx: Add a visible configuration option to the pick
  animation.

2007-01-12 22:24  mfranz

* simgear/props/: props_io.cxx: writePropeties(): create dir if
  necessary

2007-01-09 22:58  fredb

* projects/VC7.1/SimGear.vcproj,
  simgear/scene/util/SGSceneUserData.cxx: Update MSVC 7.1 project
  file and fix win32 compilation

2007-01-07 13:25  frohlich

* simgear/scene/util/: SGSceneUserData.cxx, SGSceneUserData.hxx:
  Modified Files:	 SGSceneUserData.hxx SGSceneUserData.cxx:
  Remove default argument

2007-01-07 12:53  frohlich

* simgear/scene/model/animation.cxx: Modified Files:
  animation.cxx: Change the pick animation to better handle
  different	mouse buttons.

2007-01-07 12:52  frohlich

* simgear/scene/util/: Makefile.am, SGPickCallback.hxx,
  SGSceneUserData.cxx, SGSceneUserData.hxx: Modified Files:
  SGSceneUserData.hxx SGPickCallback.hxx Makefile.am Added Files:
  SGSceneUserData.cxx: Cleanup and replace the pick callback
  with	       such a list.

2007-01-07 09:34  frohlich

* simgear/scene/model/animation.cxx: Modified Files:
  animation.cxx: Add a button argument to that animation.
  The default is to accept any mouse button.

2007-01-06 18:01  fredb

* simgear/Makefile.am: Add a prototype simgear_config.h for MSVC
  7.1 and a rule to build it.

2007-01-06 17:52  fredb

* projects/VC7.1/SimGear.vcproj, simgear/simgear_config.h-msvc71:
  Add a prototype simgear_config.h for MSVC 7.1 and a rule to build
  it.  At Olaf requests, add MSVC 8 specific symbols to remove
  pedantic warnings

2007-01-06 17:47  fredb

* Makefile.am: Remove redundant directory ( projects as a whole is
  already there )

2007-01-06 16:08  frohlich

* simgear/scene/sky/moon.cxx: Modified Files:	moon.cxx: The moo's
  state like it was with plib

2007-01-06 15:45  fredb

* SimGear.dsp: Refresh MSVC6 project file

2007-01-06 15:44  fredb

* Makefile.am: Fix a typo

2007-01-04 23:24  fredb

* projects/VC7.1/SimGear.vcproj: Add SGBinding.[ch]xx to the MSVC
  7.1 project

2007-01-04 23:23  fredb

* simgear/scene/model/animation.cxx: std::find is defined in
  <algorithm>

2007-01-04 13:55  frohlich

* simgear/scene/model/: animation.cxx, animation.hxx: Modified
  Files:   Makefile.am animation.cxx animation.hxx: Add animation
  to execute	   a command on scenery pick

2007-01-04 13:52  frohlich

* simgear/scene/util/: Makefile.am, SGNodeMasks.hxx,
  SGPickCallback.hxx, SGSceneUserData.hxx: Modified Files:
  Makefile.am SGNodeMasks.hxx Added Files:
  SGPickCallback.hxx SGSceneUserData.hxx: Preparations for generic
  scenery picking.

2007-01-04 13:51  frohlich

* simgear/props/condition.hxx: Modified Files:	condition.hxx: Also
  derive from SGReferenced

2007-01-04 13:47  frohlich

* simgear/structure/: Makefile.am, SGBinding.cxx, SGBinding.hxx,
  commands.cxx, commands.hxx: Modified Files:	 Makefile.am
  commands.cxx commands.hxx Added Files:      SGBinding.cxx
  SGBinding.hxx: Move FGBinding to SGBinding

2006-12-28 14:25  frohlich

* simgear/math/SGIntersect.hxx: Modified Files:
  SGIntersect.hxx: Make it compile with win32

2006-12-27 11:33  frohlich

* simgear/scene/tgdb/obj.cxx: Modified Files:	obj.cxx: Some kind
  of polygon offset for GL_POINTS.

2006-12-27 11:07  frohlich

* simgear/structure/: Makefile.am, SGAtomic.cxx, SGAtomic.hxx,
  SGReferenced.hxx: Modified Files:	   Makefile.am
  SGReferenced.hxx Added Files:       SGAtomic.cxx SGAtomic.hxx:
  Make the reference counts thread safe.

2006-12-27 10:23  frohlich

* simgear/math/: Makefile.am, SGBox.hxx, SGGeometry.hxx,
  SGGeometryFwd.hxx, SGGeometryTest.cxx, SGIntersect.hxx,
  SGLineSegment.hxx, SGMathTest.cxx, SGPlane.hxx, SGQuat.hxx,
  SGRay.hxx, SGSphere.hxx, SGTriangle.hxx, SGVec3.hxx: Modified
  Files:       Makefile.am SGMathTest.cxx SGQuat.hxx SGVec3.hxx
  Added Files:	 SGBox.hxx SGGeometry.hxx SGGeometryFwd.hxx
  SGGeometryTest.cxx   SGIntersect.hxx SGLineSegment.hxx
  SGPlane.hxx SGRay.hxx 	SGSphere.hxx SGTriangle.hxx:
  Small updates to the vector code, new geometry and collision
  classes for use with a bv tree to speed up collission tests.
  Also included is a rought unit test for the collissions.

2006-12-23 13:15  ehofman

* simgear/route/Makefile.am: Wether you like it or not, MispPro
  needs these libraries referenced

2006-12-17 18:52  fredb

* simgear/screen/jpgfactory.cxx: memcpy needs #include <string.h>

2006-12-16 18:29  fredb

* simgear/screen/jpgfactory.cxx, simgear/screen/jpgfactory.hxx,
  projects/VC7.1/SimGear.vcproj: Port jpgfactory to OSG

2006-12-16 14:12  fredb

* projects/VC7.1/: .cvsignore, SimGear.sln, SimGear.vcproj: Project
  files for MSVC 7.1 aka .NET 2003

2006-12-14 06:24  frohlich

* simgear/math/SGVec3.hxx: Modified Files:
  simgear/math/SGVec3.hxx: fix spelling

2006-12-08 13:22  frohlich

* simgear/scene/sky/: bbcache.cxx, bbcache.hxx, cloudfield.cxx:
  Modified Files:	  simgear/scene/sky/bbcache.cxx
  simgear/scene/sky/bbcache.hxx
  simgear/scene/sky/cloudfield.cxx

2006-12-08 13:17  frohlich

* simgear/math/SGVec3.hxx: Modified Files:	SGVec3.hxx:
  Generate any perpandicular vector to a given one.

2006-12-08 13:16  frohlich

* simgear/math/point3d.hxx: Modified Files:	point3d.hxx: Add
  explicit conversion functions to SGVec*

2006-12-05 07:14  frohlich

* simgear/scene/model/animation.cxx: Modified Files:
  simgear/scene/model/animation.cxx: Fix a problem of muliple
  texturre transform not finding the correct configuration.

2006-12-05 06:43  frohlich

* simgear/scene/model/animation.cxx: Return void instead of bool.

2006-12-03 18:44  frohlich

* simgear/screen/extensions.hxx: Modified Files:
  simgear/screen/extensions.hxx: Make it compile on macos

2006-12-03 18:27  frohlich

* simgear/scene/material/mat.cxx: Modified Files:
  simgear/scene/material/mat.cxx: Put solid scenery into the
  opaque render bin

2006-12-03 18:02  frohlich

* simgear/scene/util/SGNodeMasks.hxx: Modified Files:
  simgear/scene/util/SGNodeMasks.hxx: Add pickable bit

2006-12-03 17:57  frohlich

* simgear/scene/model/: Makefile.am, SGMaterialAnimation.cxx,
  SGMaterialAnimation.hxx, animation.cxx, animation.hxx, model.cxx,
  persparam.cxx, persparam.hxx, shadanim.cxx: Modified Files:
  simgear/scene/model/Makefile.am
  simgear/scene/model/animation.cxx
  simgear/scene/model/animation.hxx
  simgear/scene/model/model.cxx   simgear/scene/model/persparam.cxx
  simgear/scene/model/persparam.hxx
  simgear/scene/model/shadanim.cxx Added Files:
  simgear/scene/model/SGMaterialAnimation.cxx
  simgear/scene/model/SGMaterialAnimation.hxx	  Big animation
  overhaul. Improoves animation correctness.

2006-12-03 17:46  frohlich

* simgear/scene/util/SGUpdateVisitor.hxx: Modified Files:
  SGUpdateVisitor.hxx: Include light information.

2006-12-02 16:59  frohlich

* simgear/math/SGMisc.hxx: Modified Files:	   SGMisc.hxx: Add
  clip and periodic normalize functions.

2006-12-02 16:57  frohlich

* simgear/math/: SGVec2.hxx, SGVec3.hxx, SGVec4.hxx: Modified
  Files:    SGVec2.hxx SGVec3.hxx SGVec4.hxx: Add inf norm function

2006-12-02 16:56  frohlich

* simgear/math/: interpolater.cxx, interpolater.hxx: Modified
  Files:    interpolater.cxx interpolater.hxx: Enable reading
  tables directly	from our dom like tree.

2006-11-27 18:11  frohlich

* simgear/scene/sky/oursun.cxx: Modified Files:
  simgear/scene/sky/oursun.cxx: Fix the 'sun has wrong size' bug.
  Thanks to Mark Akermann.

2006-11-21 19:44  frohlich

* simgear/: math/interpolater.cxx, math/interpolater.hxx,
  props/condition.cxx, props/condition.hxx,
  scene/model/animation.cxx, scene/model/animation.hxx: Modified
  Files:       simgear/math/interpolater.cxx
  simgear/math/interpolater.hxx     simgear/props/condition.cxx
  simgear/props/condition.hxx
  simgear/scene/model/animation.cxx
  simgear/scene/model/animation.hxx: Optimize interpolation table
  lookup by using a std::map.

2006-11-21 19:39  frohlich

* simgear/math/: SGVec2.hxx, SGVec3.hxx, SGVec4.hxx: Modified
  Files:    SGVec2.hxx SGVec3.hxx SGVec4.hxx: Implement min/max for
  vectors

2006-11-20 19:19  frohlich

* simgear/scene/model/model.cxx: Modified Files:	model.cxx:
  Tweak model optimizations

2006-11-20 19:17  frohlich

* simgear/scene/model/: placementtrans.cxx, placementtrans.hxx:
  Modified Files:	  placementtrans.cxx placementtrans.hxx:
  Make use of that view 	information in the update visitor

2006-11-20 19:15  frohlich

* simgear/scene/util/SGUpdateVisitor.hxx: Modified Files:
  SGUpdateVisitor.hxx: Store some view imformation in the
  update visitor.

2006-11-18 19:58  fredb

* simgear/screen/: RenderTexture.h: Mac fix from Ima Sudonim

2006-11-14 22:09  frohlich

* simgear/scene/model/: animation.cxx, animation.hxx: Modified
  Files:   animation.hxx animation.cxx: Improove
  material/texture/blend animation

2006-11-12 11:32  frohlich

* simgear/scene/model/: animation.cxx, animation.hxx: Modified
  Files:   animation.cxx animation.hxx: Fix crash on A-10 load

2006-11-12 08:28  frohlich

* simgear/scene/model/model.cxx: Modified Files:	model.cxx:
  Leave it to osg when to do mipmapping.

2006-11-12 08:23  frohlich

* simgear/scene/model/model.cxx: Modified Files:	model.cxx:
  Reset the database path past the whole model is loaded

2006-11-12 08:22  frohlich

* simgear/math/SGQuat.hxx: Modified Files:	SGQuat.hxx: Make
  const method const

2006-11-10 06:30  frohlich

* simgear/scene/: material/mat.cxx, material/matlib.cxx,
  sky/cloud.cxx, sky/dome.cxx, sky/moon.cxx, sky/oursun.cxx,
  sky/sphere.cxx, sky/stars.cxx, tgdb/apt_signs.cxx, tgdb/leaf.cxx,
  tgdb/obj.cxx, tgdb/pt_lights.cxx, util/SGDebugDrawCallback.hxx:
  Modified Files:   simgear/scene/material/mat.cxx
  simgear/scene/material/matlib.cxx simgear/scene/sky/cloud.cxx
  simgear/scene/sky/dome.cxx simgear/scene/sky/moon.cxx
  simgear/scene/sky/oursun.cxx simgear/scene/sky/sphere.cxx
  simgear/scene/sky/stars.cxx simgear/scene/tgdb/apt_signs.cxx
  simgear/scene/tgdb/leaf.cxx simgear/scene/tgdb/leaf.hxx
  simgear/scene/tgdb/obj.cxx simgear/scene/tgdb/pt_lights.cxx
  simgear/scene/util/SGDebugDrawCallback.hxx
  simgear/screen/Makefile.am: Use color arrays for every geode.
  Just use osg::Material instead of osg::Material and the
  associated	  mode.

2006-11-09 06:42  frohlich

* simgear/scene/model/model.cxx: Modified Files:
  scene/model/model.cxx: Next step in direction liveries

2006-11-07 22:31  fredb

* simgear/math/SGQuat.hxx: copysign is already in compiler.h

2006-11-07 18:49  frohlich

* simgear/screen/: extensions.cxx, extensions.hxx: Modified Files:
  simgear/screen/extensions.cxx simgear/screen/extensions.hxx:
  Avoid      the assumption that with glx-1.4 glXGetProcAddress is
  available -	    use dlsym to get that function.

2006-11-07 08:22  fredb

* simgear/scene/model/: custtrans.cxx, custtrans.hxx: This class is
  for plib only

2006-11-07 07:40  frohlich

* simgear/scene/material/mat.cxx: Modified Files:	mat.cxx:
  Fix dark scenery problem.

2006-11-07 07:02  frohlich

* simgear/Makefile.am: Modified Files:	simgear/Makefile.am: Make
  'make distclean' work

2006-11-07 06:47  frohlich

* simgear/scene/util/SGUpdateVisitor.hxx: Modified Files:
  simgear/scene/util/SGUpdateVisitor.hxx: Only traverse active
  children.

2006-11-07 06:46  frohlich

* simgear/: scene/util/SGDebugDrawCallback.hxx, math/SGQuat.hxx:
  Modified Files:	 simgear/scene/util/SGDebugDrawCallback.hxx
  simgear/math/SGQuat.hxx: Olaf Flebbe: Make it compile on
  some more	 win32 variants.

2006-11-06 22:59  fredb

* simgear/scene/model/animation.cxx: Don't try to load textures
  when no name is given

2006-11-03 19:08  fredb

* simgear/math/SGMath.hxx: Attempt to fix the APIENTRY problem. It
  looks like a problem in OSG, or a conflict between OSG and
  plib/pui

2006-11-03 11:04  fredb

* simgear/math/SGMath.hxx: For some reason I don't know yet, the
  APIENTRY stuff in osg/GL is broken for some files. Include the
  real windows.h until we find why.

2006-11-03 10:57  fredb

* simgear/compiler.h: add copysign definition for MSVC

2006-11-02 18:40  fredb

* simgear/scene/: model/placementtrans.cxx, sky/stars.cxx: Win32
  compilation fix

2006-11-02 14:37  frohlich

* simgear/math/SGMath.hxx: Modified Files:	SGMath.hxx: Attempt
  to help IRIX builds

2006-11-01 22:25  frohlich

* simgear/math/: Makefile.am, SGMath.hxx, SGMathFwd.hxx,
  SGMatrix.hxx, SGQuat.hxx, SGVec2.hxx: Modified Files:
  Makefile.am SGMath.hxx SGMathFwd.hxx SGMatrix.hxx SGQuat.hxx
  Added Files:	     SGVec2.hxx      Improove the matrix functions.
  Improove the quaterion functions.	   Add the 2d vector.

2006-11-01 22:24  mfranz

* simgear/scene/util/.cvsignore: + .deps/

2006-10-31 07:26  frohlich

* simgear/scene/model/: Makefile.am, animation.cxx, animation.hxx,
  modellib.cxx, modellib.hxx, personality.cxx, personality.hxx:
  Modified Files:	 simgear/scene/model/Makefile.am
  simgear/scene/model/animation.cxx
  simgear/scene/model/animation.hxx
  simgear/scene/model/modellib.cxx
  simgear/scene/model/modellib.hxx Removed Files:
  simgear/scene/model/personality.cxx
  simgear/scene/model/personality.hxx:	  Updates to the animation
  system.	 Personality can be implemented easier now

2006-10-31 07:14  frohlich

* simgear/scene/util/: Makefile.am, SGStateAttributeVisitor.cxx,
  SGStateAttributeVisitor.hxx, SGTextureStateAttributeVisitor.cxx,
  SGTextureStateAttributeVisitor.hxx: Modified Files:	Makefile.am
  SGStateAttributeVisitor.hxx
  SGTextureStateAttributeVisitor.hxx Added Files:
  SGStateAttributeVisitor.cxx SGTextureStateAttributeVisitor.cxx:
  Move implementation into cxx files

2006-10-31 06:37  frohlich

* simgear/math/SGQuat.hxx: Modified Files:
  simgear/math/SGQuat.hxx: Initialize with zero not with null
  pointer

2006-10-31 06:36  frohlich

* simgear/screen/: extensions.cxx, extensions.hxx: Modified Files:
  simgear/screen/extensions.cxx simgear/screen/extensions.hxx:
  use glXGetProcAddress if approriate

2006-10-31 06:33  frohlich

* simgear/timing/timestamp.hxx: Modified Files:
  simgear/timing/timestamp.hxx: Remove reimplemented default
  implementations

2006-10-30 20:56  frohlich

* configure.ac: Modified Files: 	configure.ac: Add a
  configure flag for osg

2006-10-29 21:08  mfranz

* simgear/scene/util/.cvsignore: Makefile(.in)

2006-10-29 20:27  frohlich

* configure.ac, simgear/environment/visual_enviro.cxx,
  simgear/ephemeris/ephemeris.cxx, simgear/ephemeris/ephemeris.hxx,
  simgear/ephemeris/stardata.cxx, simgear/ephemeris/stardata.hxx,
  simgear/math/SGMatrix.hxx, simgear/math/SGQuat.hxx,
  simgear/math/SGVec3.hxx, simgear/math/SGVec4.hxx,
  simgear/scene/Makefile.am, simgear/scene/material/mat.cxx,
  simgear/scene/material/mat.hxx,
  simgear/scene/material/matlib.cxx,
  simgear/scene/material/matlib.hxx,
  simgear/scene/material/matmodel.cxx,
  simgear/scene/material/matmodel.hxx,
  simgear/scene/model/Makefile.am,
  simgear/scene/model/animation.cxx,
  simgear/scene/model/animation.hxx,
  simgear/scene/model/custtrans.hxx, simgear/scene/model/model.cxx,
  simgear/scene/model/model.hxx, simgear/scene/model/modellib.cxx,
  simgear/scene/model/modellib.hxx,
  simgear/scene/model/personality.cxx,
  simgear/scene/model/personality.hxx,
  simgear/scene/model/placement.cxx,
  simgear/scene/model/placement.hxx,
  simgear/scene/model/placementtrans.cxx,
  simgear/scene/model/placementtrans.hxx,
  simgear/scene/model/shadanim.cxx,
  simgear/scene/model/shadowvolume.hxx,
  simgear/scene/sky/cloud.cxx, simgear/scene/sky/cloud.hxx,
  simgear/scene/sky/cloudfield.cxx, simgear/scene/sky/dome.cxx,
  simgear/scene/sky/dome.hxx, simgear/scene/sky/moon.cxx,
  simgear/scene/sky/moon.hxx, simgear/scene/sky/newcloud.cxx,
  simgear/scene/sky/oursun.cxx, simgear/scene/sky/oursun.hxx,
  simgear/scene/sky/sky.cxx, simgear/scene/sky/sky.hxx,
  simgear/scene/sky/sphere.cxx, simgear/scene/sky/sphere.hxx,
  simgear/scene/sky/stars.cxx, simgear/scene/sky/stars.hxx,
  simgear/scene/tgdb/apt_signs.cxx,
  simgear/scene/tgdb/apt_signs.hxx, simgear/scene/tgdb/leaf.cxx,
  simgear/scene/tgdb/leaf.hxx, simgear/scene/tgdb/obj.cxx,
  simgear/scene/tgdb/obj.hxx, simgear/scene/tgdb/pt_lights.cxx,
  simgear/scene/tgdb/pt_lights.hxx,
  simgear/scene/tgdb/userdata.cxx, simgear/scene/tgdb/userdata.hxx,
  simgear/scene/tgdb/vasi.hxx, simgear/scene/util/Makefile.am,
  simgear/scene/util/SGDebugDrawCallback.hxx,
  simgear/scene/util/SGNodeMasks.hxx,
  simgear/scene/util/SGStateAttributeVisitor.hxx,
  simgear/scene/util/SGTextureStateAttributeVisitor.hxx,
  simgear/scene/util/SGUpdateVisitor.hxx,
  simgear/screen/jpgfactory.cxx, simgear/screen/ssgEntityArray.cxx,
  simgear/screen/ssgEntityArray.hxx, simgear/screen/tr.cxx,
  simgear/structure/Makefile.am,
  simgear/structure/ssgSharedPtr.hxx, simgear/threads/SGThread.hxx:
  Modified Files:  configure.ac
  simgear/environment/visual_enviro.cxx
  simgear/ephemeris/ephemeris.cxx
  simgear/ephemeris/ephemeris.hxx simgear/ephemeris/stardata.cxx
  simgear/ephemeris/stardata.hxx simgear/math/SGMatrix.hxx
  simgear/math/SGQuat.hxx simgear/math/SGVec3.hxx
  simgear/math/SGVec4.hxx simgear/scene/Makefile.am
  simgear/scene/material/mat.cxx simgear/scene/material/mat.hxx
  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx
  simgear/scene/material/matmodel.cxx
  simgear/scene/material/matmodel.hxx
  simgear/scene/model/Makefile.am
  simgear/scene/model/animation.cxx
  simgear/scene/model/animation.hxx
  simgear/scene/model/custtrans.hxx
  simgear/scene/model/model.cxx simgear/scene/model/model.hxx
  simgear/scene/model/modellib.cxx
  simgear/scene/model/modellib.hxx
  simgear/scene/model/personality.cxx
  simgear/scene/model/personality.hxx
  simgear/scene/model/placement.cxx
  simgear/scene/model/placement.hxx
  simgear/scene/model/placementtrans.cxx
  simgear/scene/model/placementtrans.hxx
  simgear/scene/model/shadanim.cxx
  simgear/scene/model/shadowvolume.hxx
  simgear/scene/sky/cloud.cxx simgear/scene/sky/cloud.hxx
  simgear/scene/sky/cloudfield.cxx simgear/scene/sky/dome.cxx
  simgear/scene/sky/dome.hxx simgear/scene/sky/moon.cxx
  simgear/scene/sky/moon.hxx simgear/scene/sky/newcloud.cxx
  simgear/scene/sky/oursun.cxx simgear/scene/sky/oursun.hxx
  simgear/scene/sky/sky.cxx simgear/scene/sky/sky.hxx
  simgear/scene/sky/sphere.cxx simgear/scene/sky/sphere.hxx
  simgear/scene/sky/stars.cxx simgear/scene/sky/stars.hxx
  simgear/scene/tgdb/apt_signs.cxx
  simgear/scene/tgdb/apt_signs.hxx simgear/scene/tgdb/leaf.cxx
  simgear/scene/tgdb/leaf.hxx simgear/scene/tgdb/obj.cxx
  simgear/scene/tgdb/obj.hxx simgear/scene/tgdb/pt_lights.cxx
  simgear/scene/tgdb/pt_lights.hxx
  simgear/scene/tgdb/userdata.cxx
  simgear/scene/tgdb/userdata.hxx simgear/scene/tgdb/vasi.hxx
  simgear/screen/jpgfactory.cxx simgear/screen/tr.cxx
  simgear/structure/Makefile.am simgear/threads/SGThread.hxx Added
  Files:	 simgear/scene/util/Makefile.am
  simgear/scene/util/SGDebugDrawCallback.hxx
  simgear/scene/util/SGNodeMasks.hxx
  simgear/scene/util/SGStateAttributeVisitor.hxx
  simgear/scene/util/SGTextureStateAttributeVisitor.hxx
  simgear/scene/util/SGUpdateVisitor.hxx Removed Files:
  simgear/screen/ssgEntityArray.cxx
  simgear/screen/ssgEntityArray.hxx
  simgear/structure/ssgSharedPtr.hxx	  Big BLOB on the way to
  OSG.

2006-10-29 00:12  curt

* simgear/magvar/coremag.cxx: wim van hoydonck:

  Updated to World Magnetic Model 2005.

2006-10-24 22:28  mfranz

* simgear/scene/model/model.cxx: - don't need a guarded pointer
  here - shorten variable, fix indentation

2006-10-24 21:44  mfranz

* simgear/scene/model/model.cxx: allow to switch on/off at runtime
  a whole imported <model> via <condition>:

    <model>
  <path>some/model.xml</path>
  <condition>
      <property>model/switch</property>
  </condition>
    </model>

  Of course, one could add "select" animations for all
  <object-name> in the <model>, but this is tedious and can hardly
  be done e.g. for all objects in all instruments in
  $FG_ROOT/Aircraft/Instruments-3d/ etc.

  The feature will be used in the bo105, so that civilian variants
  can have a HSI instrument, where military variants have a TACAN
  etc.

2006-10-22 21:42  durk

* simgear/: math/SGQuat.hxx, scene/sky/cloud.cxx,
  scene/sky/cloudfield.cxx, sound/soundmgr_openal.cxx: Compile time
  fixes needed to build SimGear on recent cygwin versions.

2006-10-22 21:41  durk

* configure.ac: Make configuration script compatible with
  "home-built" openal libraries on cygwin.

2006-10-22 15:08  mfranz

* simgear/props/props.cxx: - // This will come back and remove - //
  the current item each time.  Is - // that OK?

  No, it is not OK. This messes up the vector and confuses the
  iterator.  And it leads to crashes. Better read the vector in
  reverse order.

2006-10-19 05:36  curt

* simgear/scene/material/matlib.cxx: Step #1 towards abandoning the
  original point lighting scheme in favor of sprite based lighting.

2006-09-30 07:11  durk

* simgear/scene/model/placement.cxx: Memory leak fix.

2006-09-27 22:16  fredb

* simgear/debug/: logstream.cxx, logstream.hxx: Win32 only : Don't
  alloc a console when it is not needed

2006-09-02 13:21  fredb

* simgear/timing/testtimestamp.cxx: Add a simple program to
  benchmark SGTimeStamp::stamp()

2006-08-31 20:26  fredb

* simgear/timing/timestamp.cxx: Make the SGTimeStamp behave under
  Windows just like for other environments

2006-08-28 21:38  fredb

* simgear/scene/model/persparam.hxx: Use getNodeValue as initially
  planned

2006-08-28 20:53  ehofman

* simgear/scene/model/persparam.hxx: MispPro requires an explicit
  declaration.

2006-08-26 16:02  curt

* simgear/serial/serial.cxx: Frederic Bouvier:

  Make line feed behavior consistent between linux/windows.

2006-08-25 21:25  fredb

* simgear/scene/model/: modellib.cxx, personality.cxx,
  personality.hxx: Better encapsulation for personality

2006-08-25 01:03  mfranz

* simgear/scene/model/persparam.hxx: compile (gcc 4.1.0)

  ("In member function 'T SGPersonalityParameter<T>::shuffle()':
  28: error: there are no arguments to 'sg_random' that depend on a
  template parameter, so a declaration of 'sg_random' must be
  available")

2006-08-25 00:46  fredb

* simgear/scene/model/: animation.cxx, animation.hxx,
  persparam.cxx, persparam.hxx, Makefile.am: Reorganize personality
  parameters and add personality to translate, blend and scale
  animations

2006-08-08 07:05  frohlich

* simgear/math/Makefile.am: Remove duplicate linker line in the
  resulting Makefile

2006-07-30 23:02  fredb

* simgear/sound/xmlsound.cxx: Win32 fix

2006-07-30 09:48  frohlich

* simgear/: math/Makefile.am, math/SGCMath.hxx, math/SGGeodesy.hxx,
  math/SGLimits.hxx, math/SGMath.hxx, math/SGMathFwd.hxx,
  math/SGMatrix.hxx, math/SGMisc.hxx, math/SGQuat.hxx,
  math/SGVec3.hxx, math/SGVec4.hxx, math/fastmath.cxx,
  math/fastmath.hxx, sound/xmlsound.cxx: Remove fastmath funktions
  like discussed on the list.  Add a new header with forward
  declarations of the SGMath stuff.

2006-07-27 18:34  frohlich

* simgear/scene/model/location.hxx: Clean up scenery center
  handling.

2006-07-27 07:15  durk

* simgear/scene/sky/: oursun.cxx, oursun.hxx, sky.cxx, sky.hxx:
  Mark's dynamic sun color changes.

2006-07-21 17:45  curt

* simgear/scene/tgdb/: pt_lights.cxx, pt_lights.hxx: Additional
  functionality for animated point lights (i.e. approach light
  rabbits, REIL, VASI/PAPI, etc.)

  This allows the calling layer (i.e. FlightGear) better control
  over the use of OpenGL point drawing extensions.

2006-07-12 17:08  curt

* Makefile.am: Updated dist content.

2006-07-05 11:31  mfranz

* simgear/scene/sky/bbcache.cxx: remove the last redundant "delete"
  check in all of fgfs/sg (except JSBSim)

2006-07-05 04:52  andy

* simgear/nasal/: code.c, code.h, codegen.c: The previous update
  (and, embarassingly, the "nasal 1.0" release I announced on
  Freshmeat.net) was broken.  This is the proper break/continue
  fix.

2006-07-03 07:13  andy

* simgear/nasal/: code.c, code.h, codegen.c, gc.c, lib.c, vector.c:
  Been hacking at Nasal recently:

  Fix bug with break/continue inside of a foreach or forindex:
  Don't pop the vector/index inside OP_EACH, do it at the end of
  the loop.

  In the process, discovered and fixed a scary corruption issue
  with continue; it never really worked right, although simple
  usage was likely to get away without crashing.  Both the
  continue's OP_BREAK and the cleanup code at the end of a loop
  would pop the "mark" stack, leading to an underflow.	Introduced
  an OP_CONTINUE which adjusts stack but doesn't change markTop

  Re-inline the PUSH macro.  This thing is called all over the
  place from the inner loop.  If the problem is intra-expression
  side effects, then just use another expression in the macro.

  Return an empty vector when requesting zero-length subvec, not
  nil

  Have call() return the call stack in the error vector; see docs
  on plausible.org/nasal or ask Andy about this feature.

  Default closure()'s level argument to zero, not nil

  Add an optional "file name" argument to compile()

2006-07-01 22:06  mfranz

* simgear/scene/model/shadanim.cxx: actually query the <condition>
  that is already set up in SGShaderAnimation

2006-06-25 13:55  mfranz

* simgear/constants.h: add knots <-> feet-per-second conversion
  constants

2006-06-18 00:02  fredb

* simgear/io/socktest.cxx: Compile again on Win32 platforms

2006-06-17 18:04  frohlich

* simgear/math/fastmath.hxx: Make at least the header aliasing
  safe.

2006-06-17 18:04  frohlich

* simgear/scene/material/: mat.cxx, mat.hxx: Make it compile with
  gcc-3.3.6

2006-06-16 12:03  fredb

* simgear/: environment/metar.cxx, timing/geocoord.cxx: Compile
  again on Win32 platforms

2006-06-16 11:29  mfranz

* simgear/math/: fastmath.cxx, fastmath.hxx: add float_to_int()
  rounding function from Cockpit/hud_opts.hxx. The original file
  said "(c) FlightGear Project" and "probably written by Norman
  Vine".

2006-06-15 21:13  frohlich

* simgear/math/: SGVec3.hxx, SGVec4.hxx: Add dist and distSqr
  functions

2006-06-15 21:12  frohlich

* simgear/scene/model/custtrans.cxx: Remove unused extern decls

2006-06-15 10:52  frohlich

* simgear/math/: SGGeoc.hxx, SGGeod.hxx, SGVec3.hxx: Remove
  deprecated, now unused functions.

2006-06-15 10:27  frohlich

* simgear/: math/SGGeoc.hxx, math/SGGeod.hxx, math/SGMath.hxx,
  math/SGMathTest.cxx, math/SGQuat.hxx, math/SGVec3.hxx,
  math/sg_geodesy.cxx, math/sg_geodesy.hxx,
  scene/model/placement.cxx, scene/model/placement.hxx,
  timing/geocoord.cxx, timing/geocoord.h, timing/timezone.h: Small
  cleanups to the SGGeo[dc] classes, provide more hooks to use them
  directly

2006-06-15 08:14  frohlich

* simgear/screen/shader.cpp: Use function argument in va_start
  instead of local variable.

2006-06-11 15:59  frohlich

* simgear/scene/material/: matlib.cxx, matlib.hxx: Remove now
  unused function

2006-06-11 15:30  frohlich

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  material/matlib.cxx, material/matlib.hxx, tgdb/leaf.cxx,
  tgdb/obj.cxx: Modified Files:    simgear/scene/material/mat.cxx
  simgear/scene/material/mat.hxx
  simgear/scene/material/matlib.cxx
  simgear/scene/material/matlib.hxx simgear/scene/tgdb/leaf.cxx
  simgear/scene/tgdb/obj.cxx	  Attach userdata to groundtile
  scenegraph leafs that contains    a SGMaterial reference to the
  material of that leaf.    Add (physical) material properties to
  the material definitions.	    Plug a memory leak with
  GlyphSigns.

2006-06-08 07:54  frohlich

* simgear/scene/material/: matlib.cxx, matlib.hxx: Preliminary
  material lookup hooks - still unoptimized.

2006-05-24 12:16  mfranz

* simgear/props/props.cxx: whoops, sorry (Yes, it *was* tested, but
  then I made another "trivial" change and ...)

2006-05-24 11:37  mfranz

* simgear/props/props.cxx: if we are going to die we better tell
  all our listeners

2006-05-08 13:31  mfranz

* simgear/route/route.hxx: add optional position argument to
  SGRoute::add_waypoint(). Default is -1, which appends the WP like
  it used to. Valid vector indices insert the WP at this position.

2006-05-04 07:58  fredb

* simgear/screen/RenderTexture.h: Mac fix

2006-04-29 10:09  fredb

* simgear/scene/model/model.cxx: Fix the initial texture path
  problem. Loaders are setting the one given to ssgLoad as the
  default one behind our back :-(

2006-04-28 20:05  fredb

* simgear/scene/model/model.cxx: Redefine the default PLIB loader
  behavior : don't clear the texture cache after every model load

2006-04-28 17:43  mfranz

* simgear/route/route.hxx: add method to delete any waypoint (last
  waypoint if n is out of range)

2006-04-25 20:47  frohlich

* simgear/sound/soundmgr_openal.cxx: Pigeons remaining fix for the
  soundmanager crashes.

2006-04-22 15:41  mfranz

* simgear/scene/tgdb/apt_signs.cxx: thanks to Erik's texture map I
  can now drop empty.rgb altogether and just specify the same
  texture in the "foo.lighted" and "foo.unlighted" material entry.
  This also allows to drop the state cloning and thereby solves the
  most urgent apt_signs.cxx TODO. :-)

2006-04-22 11:38  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx: Add a texture cache
  mechanism. Fortunately this oly seems affective for  empty.rgb
  ...

2006-04-20 19:46  mfranz

* simgear/scene/tgdb/apt_signs.cxx: don't allow new command name to
  overwrite material name

2006-04-20 18:06  mfranz

* simgear/scene/tgdb/apt_signs.cxx: - don't use hard-coded emission
  values for unlighted signs, but load both   states from
  material.xml (separate <material> entries for now) - clone state
  less often: not once per sign element, but once per material
  switch (TODO: clone only once per material)

2006-04-20 17:20  mfranz

* simgear/scene/material/mat.cxx: fix "unknown.rgb" path (the wrong
  path was the reason why we always only got plib's lowres
  red-white chequer-board pattern along with an error message, and
  not ours ... which is much prettier, but also bigger.  (Should we
  downscale it?)

2006-04-17 13:29  mfranz

* simgear/: environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, scene/model/animation.hxx,
  scene/model/custtrans.cxx, scene/model/custtrans.hxx,
  scene/model/model.cxx, scene/model/model.hxx,
  scene/model/personality.cxx, scene/model/personality.hxx,
  scene/model/shadowvolume.cxx, scene/model/shadowvolume.hxx,
  scene/sky/cloud.cxx, scene/sky/cloud.hxx,
  scene/tgdb/pt_lights.cxx, scene/tgdb/pt_lights.hxx,
  screen/GLBitmaps.cxx, screen/GLBitmaps.h, screen/jpgfactory.cxx,
  screen/jpgfactory.hxx, screen/ssgEntityArray.cxx,
  screen/ssgEntityArray.hxx: make headers include headers they
  depend on, don't rely on the c(xx) file to do that. (This is a
  requirement for header precompiling.)

2006-04-14 16:50  mfranz

* simgear/scene/tgdb/: apt_signs.cxx, apt_signs.hxx: rename
  OBJECT_TAXI_SIGN to OBJECT_SIGN. This isn't about taxi signs any
  more, but all sorts of signs. Now is the best time to get rid of
  a misleading name.

2006-04-12 22:27  mfranz

* simgear/scene/model/: animation.cxx, animation.hxx: add
  <condition> support to textranslate & texrotate animation

2006-04-12 14:13  mfranz

* simgear/scene/tgdb/apt_signs.cxx: set sign orientation such, that
  when the sign heading=0, one looks straight on the sign face when
  looking North

2006-04-12 01:04  mfranz

* simgear/scene/tgdb/apt_signs.cxx: lower signs

2006-04-11 23:32  mfranz

* simgear/scene/tgdb/apt_signs.cxx: add minimalistic backside to
  signs as a temporary solution

2006-04-11 19:34  mfranz

* simgear/scene/tgdb/apt_signs.cxx: re-add hard-coded vertical
  distance. The coordinates should be surface points and not add
  this distance, which depends on the sign housing/hardware, after
  all.

2006-04-11 17:57  mfranz

* simgear/scene/tgdb/apt_signs.cxx: - commands do now have to start
  with @ - add commands @size, @material, @light - make "BlackSign"
  texture default - make @B, @R, @L, @Y open close their frames
  automatically (this can be   avoided by setting the @material
  manually) - add number variants for those 4 sign commands: @Y2,
  @B5, etc (according	to the spec; defaulting to the respective
  biggest panel size, i.e. @B = @B3) (detailed description will be
  added to $FG_ROOT/Docs/)

2006-04-10 18:36  mfranz

* simgear/math/: Makefile.am, linintp2.h, linintp2.inl, sphrintp.h,
  sphrintp.inl: remove obsolete files (on request by Christian
  Mayer, who has introduced them): - they are not used anywhere in
  sg/fgfs - and are very clearly *not* GPL compatible!

2006-04-10 18:21  andy

* simgear/nasal/hash.c: Manabu Nishiyama (non-FlightGear Nasal
  user) discovered an uninitialized data bug in naHash_cget().
  When the hashcode field of naStr was introduced, I forgot to set
  it in this function, which creates a temporary naStr on the
  stack.

2006-04-10 17:32  mfranz

* simgear/scene/: material/mat.cxx, material/mat.hxx,
  tgdb/apt_signs.cxx: drop xscale member again, and use xsize/ysize
  instead. (One interface element less to confuse people.)

2006-04-09 21:51  mfranz

* simgear/scene/tgdb/apt_signs.cxx: rewrite of OBJECT_TAXI_SIGN
  code. The name is a bit misleading, as this type can also create
  runway signs. (/me thinks about changing that ...)

2006-04-09 21:21  mfranz

* simgear/scene/material/: mat.cxx, mat.hxx: support for font
  textures. They are normal (but rather lenghty) <material>, but
  contain <glyph> entries with <name>, <left> and <right>. The
  latter two describe where in the texture a letter or symbol
  begins and where it ends.  (range 0-1). <xscale> defines a
  horizontal scaling factor.

2006-04-05 20:42  curt

* NEWS, configure.ac: v0.3.10 changes.

2006-03-30 16:13  mfranz

* simgear/scene/model/shadowvolume.hxx: protect ssg pointers to
  avoid occasional crashes  (of course it would be nicer if the
  Occluder would always get removed before its model branch, but
  that's not easily enforcable)

2006-03-27 20:48  curt

* Makefile.am, NEWS, configure.ac: v0.3.10-pre3 updates.

2006-03-26 10:22  mfranz

* simgear/scene/sky/cloud.cxx: If the author of this message isn't
  alerted enough to *fix* this, then I'm sure the users won't do
  that either. This is regularly triggerd and leads to meaningless
  error reports.

2006-03-25 00:24  mfranz

* simgear/: scene/model/shadowvolume.hxx, structure/event_mgr.hxx:
  more guarded pointers (we are still getting spurious crashes on
  exit because of that)

2006-03-23 22:59  curt

* Makefile.am, README.zlib, configure.ac: Updates to remove
  unneeded and old version of zlib source.

2006-03-23 17:37  curt

* NEWS, SimGear.dsp, configure.ac: v0.3.10-pre2 updates.

2006-03-23 16:39  mfranz

* simgear/environment/: visual_enviro.cxx, visual_enviro.hxx:
  Vassilii KHACHATUROV: rain fix, cleanup, read formerly hard-coded
  values		       from provided node

  "* minor redundant gl call in DrawCone2 optimized away (twice per
  frame) * corrected the glRotatef() order in drawRain even further
  (a less obvious mistake than before, which is noticed by looking
  skywards and wiggling the mouse in the view direction change
  mode) * all the "magic numbers" used in the rain rendering code
  have been provided a default (based on the old hardcoded value)
  in a form of a define, and a meaningfully named static member in
  SGEnviro"

2006-03-22 17:02  mfranz

* simgear/environment/: visual_enviro.cxx, visual_enviro.hxx:
  Vassilii KHACHATUROV: fix typos, add/fix comments, change
  variable names

2006-03-22 00:12  andy

* simgear/nasal/lib.c: Oops, implement the previously-mentioned fix
  without breaking support for omitting a length argument to mean
  "the rest of the vector".

2006-03-21 23:22  andy

* simgear/nasal/lib.c: The original code (rather oddly) interprets
  a length of zero in subvec() to mean "the whole vector".
  Melchior showed a use case (removal of the first element from a
  vector) where getting a zero-length subvector is actually
  desired.  And since I can't come up with a good reason for why
  the "feature" was there in the first place, out it goes...

2006-03-21 22:57  andy

* simgear/nasal/lib.c: Melchior discovered that cmp() was just
  wrong, failing to actually inspect the string pointers.  It also
  failed to properly sort strings where one is a prefix of the
  other.  It looks to me like I just never finished this, and it
  ended up in CVS because it just happened to compile...

2006-03-20 20:22  curt

* Doxyfile, NEWS, SimGear.dsp, SimGear.dsw, configure.ac: Updates
  for v0.3.10-pre1.

2006-03-17 20:01  mfranz

* simgear/scene/model/model.cxx: - better error message when
  submodel loading failed - use alignmainmodel node in callback
  (not used anywhere yet)

2006-03-16 19:01  andy

* simgear/nasal/iolib.c: The handle gets nulled out if the user has
  closed the file; don't pass that null to fclose() in a garbage
  collection destructor too...

2006-03-15 20:42  andy

* simgear/nasal/: Makefile.am, bitslib.c, iolib.c: Fix broken
  checkin in iolib.c.  Also add the 'bits' library, which has a
  buf() function needed to make convenient use of io.read().

2006-03-15 20:35  andy

* simgear/nasal/iolib.c: Melchior found the first bug -- report EOF
  as nil in readln().

2006-03-15 19:09  andy

* simgear/nasal/: Makefile.am, iolib.c, iolib.h: Add the Nasal I/O
  library so Melchior can play with it.  Not enabled currently (see
  NasalSys.cxx in the flightgear CVS)

2006-03-14 16:58  mfranz

* simgear/scene/: model/shadowvolume.cxx, sky/bbcache.cxx:
  --warnings

2006-03-14 16:55  mfranz

* simgear/props/: props.cxx, props.hxx: --warnings

2006-03-14 16:49  mfranz

* simgear/io/sg_binobj.cxx: --warnings

2006-03-14 13:58  mfranz

* simgear/timing/timezone.cxx: close zone.tab file after having
  read all entries

2006-03-14 11:38  mfranz

* simgear/props/: condition.cxx, condition.hxx: prevent animations
  from losing nodes, because other processes removed them (We are
  currently getting a lot of aborts in the condition code when
  running MP. I don't expect this to fix it, but a bug is a bug.)

  (reviewed by Fred, who also fixed *my* bugs :-)

2006-03-12 19:56  mfranz

* simgear/scene/model/model.cxx: Better use a node that is very
  clearly not used by other services (e.g. animations). The data
  class doesn't mind.

2006-03-12 11:09  mfranz

* simgear/scene/model/model.cxx: activate model load/unload
  callback again. It turned out *not* to be the cause for the MP
  crashes -- the same crashes did still occur without it.

2006-03-11 23:20  ehofman

* simgear/screen/: RenderTexture.cpp, RenderTexture.h: Mac OS X
  fixes from Ima Sudonim.

2006-03-10 23:58  mfranz

* simgear/scene/model/model.cxx: commenting out Nasal in scenery
  models for now. This could be responsible for an MP/AI crash.
  Still investigating.

2006-03-09 17:17  mfranz

* simgear/scene/model/animation.cxx: warning--

2006-03-09 10:54  ehofman

* simgear/screen/: RenderTexture.cpp, RenderTexture.h,
  extensions.hxx: Alexander Powell:

  Add MAC OS X Render Texture support:

  Most texture modes seem to work on my Powerbook, but I don't have
  a wide array of machines to test it on otherwise.

  If you have problems, please let me know and I'll see if I can
  help track down the source of the bug.  I'd love to keep working
  on it if time permits (I use RenderTexture in a few other
  projects), so I'll keep you informed if there are any changes
  that I make for the better.

2006-03-09 10:03  mfranz

* simgear/scene/: material/matmodel.cxx, model/model.cxx,
  model/model.hxx, model/modellib.cxx, model/modellib.hxx:
  model.[ch]xx:   add abstract class SGModelData. If a pointer to
  such a class is handed over	to sgLoad3DModel, then its
  modelLoaded() method is called with path, property   node and
  branch. And then it's added to the scene graph so that it's
  destroyed when the model branch is removed from the graph.

  modellib.[ch]xx:   only cache objects when asked to. This is the
  case for OBJECT_SHARED   and random objects (like before), but no
  longer for OBJECT_STATIC.    These are now removed from the graph
  when they are "out of sight". This	prevents accumulation of
  static models, and makes destroying model data    possible (e.g.
  removing Nasal modules)

  matmodel.cxx:   set cache flag for random objects (same behavior
  as before)

2006-03-08 19:16  mfranz

* simgear/: compiler.h, constants.h, sg_inlines.h,
  bucket/newbucket.cxx, bucket/newbucket.hxx, debug/logstream.cxx,
  debug/logstream.hxx, environment/metar.cxx,
  environment/metar.hxx, environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, ephemeris/celestialBody.cxx,
  ephemeris/celestialBody.hxx, ephemeris/ephemeris.cxx,
  ephemeris/ephemeris.hxx, ephemeris/jupiter.cxx,
  ephemeris/jupiter.hxx, ephemeris/mars.cxx, ephemeris/mars.hxx,
  ephemeris/mercury.cxx, ephemeris/mercury.hxx,
  ephemeris/moonpos.cxx, ephemeris/moonpos.hxx,
  ephemeris/neptune.cxx, ephemeris/neptune.hxx,
  ephemeris/pluto.hxx, ephemeris/saturn.cxx, ephemeris/saturn.hxx,
  ephemeris/star.cxx, ephemeris/star.hxx, ephemeris/stardata.cxx,
  ephemeris/stardata.hxx, ephemeris/uranus.cxx,
  ephemeris/uranus.hxx, ephemeris/venus.cxx, ephemeris/venus.hxx,
  io/iochannel.cxx, io/iochannel.hxx, io/lowlevel.cxx,
  io/lowlevel.hxx, io/sg_binobj.cxx, io/sg_binobj.hxx,
  io/sg_file.cxx, io/sg_file.hxx, io/sg_serial.cxx,
  io/sg_serial.hxx, io/sg_socket.cxx, io/sg_socket.hxx,
  io/sg_socket_udp.cxx, io/sg_socket_udp.hxx, magvar/coremag.cxx,
  magvar/coremag.hxx, magvar/magvar.cxx, magvar/magvar.hxx,
  math/SGGeoc.hxx, math/SGGeod.hxx, math/SGGeodesy.cxx,
  math/SGGeodesy.hxx, math/SGLimits.hxx, math/SGMath.hxx,
  math/SGMathTest.cxx, math/SGMatrix.hxx, math/SGMisc.hxx,
  math/SGQuat.hxx, math/SGVec3.hxx, math/SGVec4.hxx,
  math/fastmath.hxx, math/interpolater.cxx, math/interpolater.hxx,
  math/leastsqs.cxx, math/leastsqs.hxx, math/point3d.hxx,
  math/polar3d.cxx, math/polar3d.hxx, math/sg_memory.h,
  math/sg_random.c, math/sg_random.h, math/sg_types.hxx,
  math/vector.cxx, math/vector.hxx, misc/interpolator.cxx,
  misc/interpolator.hxx, misc/sg_path.cxx, misc/sg_path.hxx,
  misc/sgstream.cxx, misc/sgstream.hxx, misc/stopwatch.hxx,
  misc/strutils.cxx, misc/strutils.hxx, misc/tabbed_values.cxx,
  misc/tabbed_values.hxx, misc/texcoord.cxx, misc/texcoord.hxx,
  misc/zfstream.cxx, misc/zfstream.hxx, props/props_io.cxx,
  route/route.cxx, route/route.hxx, route/waypoint.cxx,
  route/waypoint.hxx, scene/material/mat.cxx,
  scene/material/mat.hxx, scene/material/matlib.cxx,
  scene/material/matlib.hxx, scene/material/matmodel.cxx,
  scene/material/matmodel.hxx, scene/model/location.cxx,
  scene/model/location.hxx, scene/model/placementtrans.cxx,
  scene/model/placementtrans.hxx, scene/model/shadanim.cxx,
  scene/model/shadowvolume.cxx, scene/model/shadowvolume.hxx,
  scene/sky/bbcache.cxx, scene/sky/bbcache.hxx,
  scene/sky/cloud.cxx, scene/sky/cloud.hxx,
  scene/sky/cloudfield.cxx, scene/sky/cloudfield.hxx,
  scene/sky/dome.cxx, scene/sky/dome.hxx, scene/sky/moon.cxx,
  scene/sky/moon.hxx, scene/sky/newcloud.cxx,
  scene/sky/newcloud.hxx, scene/sky/oursun.cxx,
  scene/sky/oursun.hxx, scene/sky/sky.cxx, scene/sky/sky.hxx,
  scene/sky/sphere.cxx, scene/sky/sphere.hxx, scene/sky/stars.cxx,
  scene/sky/stars.hxx, scene/tgdb/apt_signs.cxx,
  scene/tgdb/apt_signs.hxx, scene/tgdb/leaf.cxx,
  scene/tgdb/leaf.hxx, scene/tgdb/obj.cxx, scene/tgdb/obj.hxx,
  scene/tgdb/pt_lights.cxx, scene/tgdb/pt_lights.hxx,
  scene/tgdb/userdata.cxx, scene/tgdb/userdata.hxx,
  scene/tgdb/vasi.hxx, screen/colors.hxx, screen/colours.h,
  screen/jpgfactory.cxx, screen/jpgfactory.hxx,
  screen/screen-dump.cxx, screen/screen-dump.hxx,
  screen/shader.cpp, screen/shader.h, screen/ssgEntityArray.cxx,
  serial/serial.cxx, serial/serial.hxx, sound/sample_openal.cxx,
  sound/sample_openal.hxx, sound/soundmgr_openal.cxx,
  sound/soundmgr_openal.hxx, sound/xmlsound.cxx,
  sound/xmlsound.hxx, structure/SGReferenced.hxx,
  structure/SGSharedPtr.hxx, structure/callback.hxx,
  structure/ssgSharedPtr.hxx, structure/subsystem_mgr.hxx,
  threads/SGThread.hxx, timing/geocoord.cxx, timing/geocoord.h,
  timing/lowleveltime.cxx, timing/lowleveltime.h,
  timing/sg_time.cxx, timing/sg_time.hxx, timing/timestamp.cxx,
  timing/timestamp.hxx, timing/timezone.cxx, timing/timezone.h,
  xml/easyxml.cxx, xml/hashtable.c:
  - new FSF addresses - coplied license headers from h(xx) files to
  respective c(xx) files - removed trailing spaces - fixe $Id$ -
  fixed typos

2006-03-04 14:27  david

* simgear/: math/.cvsignore, screen/.cvsignore: Ignore generated
  binaries.

2006-03-04 13:46  ehofman

* simgear/props/props.cxx: Mathias Fr�hlich:

  zero out all parent pointers, else they might be dangling.

2006-03-03 16:11  ehofman

* simgear/sound/xmlsound.cxx: Mathias Fr�hlich:

  I have some bugfixes which will avoid fg just crashing if the
  sound device could not be opened.

2006-02-26 12:02  fredb

* simgear/: math/sg_memory.h, misc/stopwatch.hxx: Wrong config file
  name

2006-02-22 21:50  andy

* simgear/scene/model/: animation.cxx, animation.hxx: Fix from
  Melchior: Set static values at every condition change, not only
  initially.

2006-02-21 13:59  fredb

* simgear/: structure/subsystem_mgr.cxx, threads/SGQueue.hxx:
  Melchior FRANZ:

  - don't unlock an already unlocked mutex (Someone wanted to be on
    the safe side with this, but the result is undefined and makes
    pthread_mutex_destroy fail. Reference: manpage for
  pthread_mutexattr_gettype/The Open Group[1]: "Attempting to
  unlock a mutex of this type which is not locked results in
  undefined behaviour.")

  - re-enabled all subsystem destructors again (this has been
  disabled   because fgfs hung on exit, due to the mutex destroy
  failure from	 above.)

  Reference:
  http://www.opengroup.org/onlinepubs/007908799/xsh/pthread_mutexattr_gettype.html

2006-02-21 11:47  ehofman

* simgear/: compiler.h, constants.h, sg_inlines.h, version.h.in,
  bucket/newbucket.cxx, bucket/newbucket.hxx, debug/logstream.cxx,
  debug/logstream.hxx, environment/metar.cxx,
  environment/metar.hxx, environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, ephemeris/celestialBody.cxx,
  ephemeris/celestialBody.hxx, ephemeris/ephemeris.cxx,
  ephemeris/ephemeris.hxx, ephemeris/jupiter.cxx,
  ephemeris/jupiter.hxx, ephemeris/mars.cxx, ephemeris/mars.hxx,
  ephemeris/mercury.cxx, ephemeris/mercury.hxx,
  ephemeris/moonpos.cxx, ephemeris/moonpos.hxx,
  ephemeris/neptune.cxx, ephemeris/neptune.hxx,
  ephemeris/pluto.hxx, ephemeris/saturn.cxx, ephemeris/saturn.hxx,
  ephemeris/star.cxx, ephemeris/star.hxx, ephemeris/stardata.cxx,
  ephemeris/stardata.hxx, ephemeris/uranus.cxx,
  ephemeris/uranus.hxx, ephemeris/venus.cxx, ephemeris/venus.hxx,
  io/iochannel.cxx, io/iochannel.hxx, io/lowlevel.cxx,
  io/lowlevel.hxx, io/sg_binobj.cxx, io/sg_binobj.hxx,
  io/sg_file.cxx, io/sg_file.hxx, io/sg_serial.cxx,
  io/sg_serial.hxx, io/sg_socket.cxx, io/sg_socket.hxx,
  io/sg_socket_udp.cxx, io/sg_socket_udp.hxx, magvar/coremag.cxx,
  magvar/coremag.hxx, magvar/magvar.cxx, magvar/magvar.hxx,
  math/.cvsignore, math/SGGeoc.hxx, math/SGGeod.hxx,
  math/SGGeodesy.cxx, math/SGGeodesy.hxx, math/fastmath.cxx,
  math/interpolater.cxx, math/interpolater.hxx, math/leastsqs.cxx,
  math/leastsqs.hxx, math/sg_random.c, math/sg_random.h,
  math/sg_types.hxx, math/vector.cxx, math/vector.hxx,
  misc/texcoord.hxx, route/waypoint.hxx, scene/sky/oursun.cxx,
  scene/sky/sphere.cxx, scene/sky/stars.cxx,
  scene/tgdb/apt_signs.hxx, scene/tgdb/leaf.cxx,
  scene/tgdb/leaf.hxx, scene/tgdb/obj.cxx, scene/tgdb/obj.hxx,
  scene/tgdb/pt_lights.hxx, scene/tgdb/userdata.hxx,
  scene/tgdb/vasi.hxx, screen/screen-dump.cxx,
  screen/screen-dump.hxx, screen/tr.cxx, screen/tr.h,
  serial/serial.cxx, serial/serial.hxx,
  structure/subsystem_mgr.cxx, threads/SGQueue.hxx,
  timing/sg_time.cxx, timing/sg_time.hxx, timing/timestamp.hxx:
  Back out the previous patch.

2006-02-21 10:48  ehofman

* simgear/math/: SGGeoc.hxx, SGGeod.hxx: Declare specified
  functions, otherwise MIPSpro bails out.

2006-02-21 10:40  ehofman

* simgear/: compiler.h, constants.h, sg_inlines.h,
  bucket/newbucket.cxx, bucket/newbucket.hxx, debug/logstream.cxx,
  debug/logstream.hxx, environment/metar.cxx,
  environment/metar.hxx, environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, ephemeris/celestialBody.cxx,
  ephemeris/celestialBody.hxx, ephemeris/ephemeris.cxx,
  ephemeris/ephemeris.hxx, ephemeris/jupiter.cxx,
  ephemeris/jupiter.hxx, ephemeris/mars.cxx, ephemeris/mars.hxx,
  ephemeris/mercury.cxx, ephemeris/mercury.hxx,
  ephemeris/moonpos.cxx, ephemeris/moonpos.hxx,
  ephemeris/neptune.cxx, ephemeris/neptune.hxx,
  ephemeris/pluto.hxx, ephemeris/saturn.cxx, ephemeris/saturn.hxx,
  ephemeris/star.cxx, ephemeris/star.hxx, ephemeris/stardata.cxx,
  ephemeris/stardata.hxx, ephemeris/uranus.cxx,
  ephemeris/uranus.hxx, ephemeris/venus.cxx, ephemeris/venus.hxx,
  io/iochannel.cxx, io/iochannel.hxx, io/lowlevel.cxx,
  io/lowlevel.hxx, io/sg_binobj.cxx, io/sg_binobj.hxx,
  io/sg_file.cxx, io/sg_file.hxx, io/sg_serial.cxx,
  io/sg_serial.hxx, io/sg_socket.cxx, io/sg_socket.hxx,
  io/sg_socket_udp.cxx, io/sg_socket_udp.hxx, magvar/coremag.cxx,
  magvar/coremag.hxx, magvar/magvar.cxx, magvar/magvar.hxx,
  math/.cvsignore, math/SGGeoc.hxx, math/SGGeod.hxx,
  math/SGGeodesy.cxx, math/SGGeodesy.hxx,
  structure/subsystem_mgr.cxx, threads/SGQueue.hxx: Melchior FRANZ:

  - new FSF address - removed a few hundred trailing spaces - fixed
  a few $Id$ lines - copied two license headers from *.hxx files to
  their respective   *.cxx counterparts - added two test aps to
  .cvsignore - don't unlock an already unlocked mutex (Someone
  wanted to be on   the safe side with this, but the result is
  undefined and makes	pthread_mutex_destroy fail. Reference:
  manpage for	pthread_mutexattr_gettype/The Open Group[1]:
  "Attempting to   unlock a mutex of this type which is not locked
  results in   undefined behaviour.") - re-enabled all subsystem
  destructors again (this has been disabled   because fgfs hung on
  exit, due to the mutex destroy failure)

2006-02-20 16:12  ehofman

* simgear/math/: SGGeoc.hxx, SGGeod.hxx, SGGeodesy.cxx,
  SGGeodesy.hxx, SGLimits.hxx, SGMath.hxx, SGMathTest.cxx,
  SGMatrix.hxx, SGMisc.hxx, SGQuat.hxx, SGVec3.hxx, SGVec4.hxx:
  Mathias Froehlich: Add license information.

2006-02-19 18:22  ehofman

* simgear/math/: SGMathTest.cxx, SGMisc.hxx, SGQuat.hxx,
  SGVec3.hxx, SGVec4.hxx: Mathias Fr�hlich:

  This patch makes use of the vectors now available in simgear with
  that past patch. And using that it simplyfies the carrier code
  somehow.

  - Small additional factory's to the quaternion code are done in
  the simgear	part. Also more explicit unit names in the factory
  functions.  - The flightgear part makes use of them and
  simplyfies some computations	 especially in the carrier code.  -
  The data part fixes the coordinate frames I used for the park
  positions in	 the carrier to match the usual ones. I believed
  that I had done so, but it   was definitly different. Also there
  are more parking positions avaliable now.

2006-02-19 18:13  ehofman

* simgear/environment/metar.cxx: Melchior FRANZ:

  - change SG back to FG (this was accidently changed in three
  wrong   places when I prepared the file for SG) - correct length
  for the proxy id detection - set (guessed) deposit depth < 1mm
  correctly - set deposit type string - formatting

2006-02-18 17:19  fredb

* simgear/simgear_config.h.vc5: Missing defines

2006-02-18 14:24  fredb

* simgear/: misc/texcoord.cxx, route/waypoint.cxx,
  scene/model/animation.cxx, scene/model/modellib.cxx,
  scene/model/shadanim.cxx, scene/model/shadowvolume.cxx,
  scene/sky/cloud.cxx, scene/sky/sky.cxx, scene/tgdb/apt_signs.cxx,
  scene/tgdb/pt_lights.cxx, scene/tgdb/userdata.cxx,
  sound/sample_openal.cxx, sound/soundmgr_openal.cxx,
  io/sg_socket.cxx, math/SGGeodesy.cxx, math/SGMathTest.cxx,
  math/polar3d.cxx, math/sg_geodesy.cxx, props/props_io.cxx: Add
  missing include files needed by the new math code under windows

2006-02-18 13:04  fredb

* SimGear.dsp: Fix Code generation option for debug build

2006-02-17 22:57  fredb

* simgear/math/: SGMatrix.hxx, SGQuat.hxx, SGVec3.hxx, SGVec4.hxx:
  Remove compiler warnings

2006-02-17 16:13  curt

* simgear/: environment/metar.cxx, environment/visual_enviro.cxx,
  io/sg_binobj.cxx, misc/sg_path.cxx, props/props_io.cxx,
  scene/model/location.cxx, scene/model/shadanim.cxx,
  scene/model/shadowvolume.cxx, scene/sky/cloudfield.cxx: Clean up
  some compiler warnings.

2006-02-17 10:59  ehofman

* simgear/math/localconsts.hxx: Remove unused files.

2006-02-17 10:23  ehofman

* simgear/math/fastmath.hxx: Melchior FRANZ:

  This patch fixes the sound of 737, Concorde and others, if fgfs
  was compiled with newer gcc versions (e.g. gcc 4.0.2). These
  compilers implement the c++ standard more strictly, and thus
  don't guarantee that c-style casted pointers to different data
  types really point to the same address. This is only guaranteed
  for union members.

2006-02-17 10:22  ehofman

* simgear/math/: Makefile.am, SGGeoc.hxx, SGGeod.hxx,
  SGGeodesy.cxx, SGGeodesy.hxx, SGLimits.hxx, SGMath.hxx,
  SGMathTest.cxx, SGMatrix.hxx, SGMisc.hxx, SGQuat.hxx, SGVec3.hxx,
  SGVec4.hxx, point3d.hxx, polar3d.cxx, polar3d.hxx,
  sg_geodesy.cxx, sg_geodesy.hxx: Mathias Fr�hlich:

  The patch adds a vector lib I have put together during the last
  time, it is just handy and interfaces well with s(s)g*. Together
  with some small modifications this will later also interface well
  with OpenSceneGraphs vectors/matrices. Using this vector kernel
  is targeted to have a handy matrix/vector lib available and to
  provide a scenegraph independent vector type for use with a small
  scenegraph wrapper aimed for a smooth transition to
  openscenegraph.

  That vector code also includes an improoved geodetic conversion
  routine I have found some time ago published in the 'journal of
  geodesy' which avoids iterative computations for that purpose.

  Also the geodetic position class is more typesafe and unitsafe
  than the Point3D currently is.

  That part is relatively old and in use in my local trees for
  several months now.

2006-02-17 09:58  ehofman

* SimGear.dsp, SimGear.dsw, am2dsp.cfg, simgear/constants.h,
  simgear/math/fastmath.hxx, simgear/nasal/mathlib.c: Olaf Flebbe:

  This patch makesFlightGear at least compile on MSVC. I hope I
  have removed reference of my other local changes. DSP and DSW
  files are included for reference. They have been reconstructed
  with am2dsp.pl. I had to introduce a change to am2dsp because of
  the need of filenames with embedded spaces. (Yuck)

  The major direction is to remove clutter like the
  _USE_MATH_DEFINES and have it on the compiler command line sice
  there is no central include file. You will have to put it on the
  command line for your locale Project files, if it not there,
  already. I added the _CRT_SECURE_NO_DEPRECATE define for 2005,
  since it does no harm to other VC version.

  Third Party Libs like plib, OpenALSDK, freeglut, pthreads-win32,
  zlib are unpacked as is side by side. Only change put the
  includes of OpenAL into include/AL rather directly into include.

2006-02-07 21:50  fredb

* simgear/props/props.cxx: Outputing 6 digits is not enough for a
  double

2006-02-02 10:56  ehofman

* simgear/environment/visual_enviro.cxx: Vassilii Khachaturov:

  Fix the current buggy rain orientation behaviour for the views
  attached to the aircraft (while still inheriting bugs with the
  views attached to anything else).

2006-01-31 16:14  ehofman

* simgear/scene/sky/: oursun.hxx, sky.hxx: Expose the sun halo
  texture handle.

2006-01-31 16:13  ehofman

* simgear/screen/extensions.hxx: Expose GL_COORD_REPLACE

2006-01-30 21:30  ehofman

* simgear/sound/xmlsound.cxx: create an absolute value before
  calling log or log10, this adds support for sound on negative
  numbers (thrust reverse for example).

2006-01-30 11:56  ehofman

* simgear/props/: props.cxx, props.hxx: Melchior FRANZ:

  add optional arg to SGPropertyNode::addChangeListener that
  triggers the listener function call initially. This is useful for
  cases where a freshly installed listener wants to treat the
  current property value as changed from 'unknown' to the actual
  value right away.

  Examples can be found in the Nasal incarnation setlistener(),
  where we have for example this (in $FG_ROOT/Nasal/gui.nas):

    INIT = func {
  ...
  setlistener("/sim/rendering/fps-display", fpsDisplay);
  if (getprop("/sim/rendering/fps-display")) {
      fgcommand("dialog-show", props.Node.new({"dialog-name":
  "fps"}));
  }
    }

  That is: we first attach a listener that cares for changes to the
  FPS display switch, but then we have to manually open the dialog
  initially.  That's a duplication of code and could be as simple
  as this (INIT part only):

    INIT = func {
  ...
  setlistener("/sim/rendering/fps-display", fpsDisplay, 1);
    }

  That is: the optional third arg makes fpsDisplay be called
  initially, and then again with every write action. My first
  solution was in the Nasal code only, but Andy (rightfully) says
  that this should rather be in sg.

2006-01-27 16:27  ehofman

* simgear/environment/visual_enviro.cxx: On a second thought,
  disable smooth shaded lines for all segments of the lightning.

2006-01-27 16:18  ehofman

* simgear/environment/visual_enviro.cxx: Disable smooth lines for
  certain lightning segments.

2006-01-26 10:15  ehofman

* simgear/screen/extensions.hxx: Add support for point sprites.

2006-01-24 22:49  fredb

* simgear/sound/soundmgr_openal.cxx: The sample is now managed by a
  SGSharedPtr and shouldn't be deleted explicitely

2006-01-24 15:44  ehofman

* simgear/: environment/visual_enviro.cxx, scene/material/mat.cxx,
  scene/material/mat.hxx, scene/material/matlib.cxx,
  scene/material/matlib.hxx, scene/material/matmodel.cxx,
  scene/material/matmodel.hxx, scene/model/placement.hxx,
  scene/sky/cloud.cxx, scene/sky/newcloud.cxx,
  sound/sample_openal.hxx, sound/soundmgr_openal.cxx,
  sound/soundmgr_openal.hxx, sound/xmlsound.cxx,
  sound/xmlsound.hxx: Mathias Fr�hlich:

  Incorporating the shared ptr code: - All scenegraph references
  from SimGear - SGMaterial which already had a reference counter
  uses now that common	 infrastructure.  - SGMatModel is now
  counted.  - SGSoundSample from SimGear - And the corresponding
  change for the sound samples in flightgear which fixes   a latent
  crash if FGBeacon would evern be deleted.

2006-01-12 14:47  ehofman

* simgear/: props/props.cxx, props/props.hxx,
  scene/model/modellib.cxx, scene/model/modellib.hxx: Mathias
  Fr�hlich:

  Use the new automatic reference counter instead of doing that
  ourselfes.

2006-01-10 21:25  fredb

* simgear/nasal/lib.c: MSVC vsnprintf ( in fact _vsnprinft )
  returns -1 when the buffer is too short

2006-01-07 14:21  fredb

* simgear/io/sg_binobj.cxx: Use the new SGPath::create_dir function
  Ensure no triangles array could have more than 32767 vertices, a
  PLIB limit.

2006-01-04 17:44  curt

* simgear/scene/model/placement.hxx: John Ellson:

  This patch fixes this SimGear compile error on x86_64 Fedora
  Development with gcc-4.1:

  placement.hxx:49: error: extra qualification
  ‘SGModelPlacement::’ on member ‘init’

2006-01-04 10:08  ehofman

* simgear/misc/sg_path.cxx: MinGW fixes.

2006-01-03 18:40  ehofman

* simgear/nasal/code.c: Make the sgi code the default to prevent
  future problems.

2006-01-03 18:34  ehofman

* simgear/bucket/newbucket.hxx: Save some memory.

2006-01-02 14:32  ehofman

* simgear/nasal/code.c: Fix the persisent IRIX bug.

2006-01-02 11:04  ehofman

* simgear/nasal/: hash.c, vector.c: prevent confusion by not using
  a standard name.

2005-12-29 13:00  ehofman

* simgear/structure/: Makefile.am, SGReferenced.hxx,
  SGSharedPtr.hxx, ssgSharedPtr.hxx: Mathias Fr�hlich:

  Add the basic infrastructure for a reference counter class.
  Adding it now (without using it) enables Mathias and others to
  prepare some memory reduction code.

2005-12-19 13:52  ehofman

* simgear/: environment/visual_enviro.cxx, io/sg_binobj.cxx:
  Vassilii Khachaturov:

  clean up some build warnings caught with gcc-4.0.

2005-12-19 11:22  ehofman

* simgear/misc/: sg_path.cxx, sg_path.hxx: Frederic Bouvier:

  Fix a problem where the directory being created is made relative
  when in fact it's absolute. It also tightens things a bit on the
  Windows side.

  Erik:

  Make the section that splits up the directory in a lists of
  parent directories a standalone function.

2005-12-18 10:37  ehofman

* simgear/misc/sg_path.cxx: Frederic: this patch to sg_path.cxx
  will filter out false alarms when directory already exists.

2005-12-17 23:12  ehofman

* simgear/misc/sg_path.cxx: Frederic Bouvier:

  The create_dir was totally broken. No function was used at the
  right place except mkdir. This patch now create directories
  without segfaulting.

  Erik:

  This was my bad, I've been using a really slow computer for the
  last ten months and recompiling SimGear with a change to the
  properties code takes ages, so once in a while I apply something
  not entirely tested. This is one really bad example which
  shouldn't have happened. Thanks to Frederic for fixing it.

2005-12-17 16:41  ehofman

* simgear/misc/sg_path.cxx: Add the subdir to the path before
  trying to create it, instead of afterwards.

2005-12-17 16:32  ehofman

* simgear/misc/: sg_path.cxx, sg_path.hxx: MSVC fixes.

2005-12-17 16:15  ehofman

* configure.ac: Try to detect the proper type for mode_t

2005-12-17 16:11  ehofman

* simgear/props/: props.hxx, props_io.cxx, props_io.hxx: Stefan
  Seifert: Add the posibility to specify a userarchive attribute
  which could be used to save user prefferences at program exit.

2005-12-17 16:06  ehofman

* simgear/misc/: sg_path.cxx, sg_path.hxx: Add a function to create
  aa new directory

2005-12-14 11:28  ehofman

* configure.ac, simgear/screen/Makefile.am,
  simgear/screen/RenderTexture.cpp: Mathias Fr�hlich:

  Detect whether we support pubuffers at runtime using the GLX
  extension string.

  Erik:

  Add support to compile TestRenderTexture if GLUT is installed.

2005-12-11 14:41  ehofman

* simgear/screen/TestRenderTexture.cpp: MacOs X fix.

2005-12-11 14:35  ehofman

* simgear/: environment/metar.cxx, scene/material/matlib.cxx,
  scene/model/model.cxx, sound/sample_openal.cxx, xml/easyxml.cxx:
  Vassilii Khachaturov:

  * in some cases more specific sg exception types were used in
  place   of the more generic one, e.g., sg_io_exception instead of
  sg_exception	 when the context of the error was an IO error * in
  some cases, the error message was made more specific * minor
  style fix for exception rethrowing --- using throw; whenever	 a
  re-throw is made; sometimes optimizing away the exception symbol
  name	 in the catch handler at all * more specific catch handlers
  added in some places -- e.g.,   an sg_io_exception caught ahead
  of sg_exception

2005-12-11 14:26  ehofman

* simgear/misc/.cvsignore: add swap_test to .cvsignore

2005-12-11 13:51  ehofman

* simgear/debug/logstream.hxx: Add a proper return statement for
  MSVC.

2005-12-06 19:45  ehofman

* Doxyfile: Vassilii: help those using the Doxygen docs.

2005-12-06 19:29  ehofman

* simgear/scene/model/placementtrans.cxx: Mathias: silence some
  valgrind warnings so that you can concentrate better on the real
  problems.

2005-11-27 10:48  ehofman

* simgear/screen/RenderTexture.cpp: How did this end up there??

2005-11-27 10:46  ehofman

* simgear/screen/: RenderTexture.cpp, TestRenderTexture.cpp:
  Initialize glut before using it.

2005-11-23 10:28  ehofman

* simgear/screen/RenderTexture.cpp: More WIN32 fixes.

2005-11-22 19:42  ehofman

* simgear/screen/RenderTexture.cpp: Oops, WIN32 typo fix.

2005-11-22 19:03  ehofman

* simgear/screen/RenderTexture.cpp:

  * Use SimGear's logging facility isntead of printf's.  * Hopefuly
  fix a nasty (mostly ATI related) bug.

2005-11-17 21:54  curt

* NEWS, configure.ac: Changes for v0.3.9 (final).

2005-11-17 16:30  curt

* simgear/timing/sg_time.hxx: Add a small accessor function to
  expose local timezone offset.

2005-11-15 22:43  curt

* simgear/timing/sg_time.hxx: Fix a small spelling mistake.

2005-11-14 19:25  ehofman

* simgear/threads/SGThread.hxx: Revert to the original (0.9.8)
  version, it causes more problems than it solves (did actually
  solve any?)

2005-11-13 10:42  ehofman

* simgear/misc/stdint.hxx: Put in public domain, Curtis wants it
  (because net_fdm.hxx depends on it) and I created the other
  functions.

2005-11-12 13:22  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx: Let the
  application free the buffer data.

2005-11-12 13:17  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx: Prevent a
  possible memory leak.

2005-11-12 11:55  ehofman

* simgear/sound/sample_openal.cxx: add a missing character.

2005-11-12 11:26  ehofman

* simgear/sound/: openal_test2.cxx, sample_openal.cxx,
  sample_openal.hxx, xmlsound.cxx: Make a clear separation between
  loading a sound file into main memroy and sending it to the
  driver. This prevents data to be loaded into the main memory (or
  onto the soundcard's memory) when it's not needed.

2005-11-11 14:19  ehofman

* simgear/ephemeris/: star.cxx, star.hxx: Expose some internals.

2005-11-11 01:44  curt

* NEWS, configure.ac: v0.3.9-pre3 updates.

2005-11-10 10:57  ehofman

* simgear/debug/logstream.hxx: MSVC fix.

2005-11-10 10:55  ehofman

* simgear/structure/event_mgr.hxx: gcc 4.0 fix.

2005-11-09 21:34  andy

* simgear/structure/: event_mgr.cxx, event_mgr.hxx: Architectural
  fix allowing the "tip" popups (FOV, view name, etc...) to pop
  themselves down while the simulator is paused.

  The problem was with the "real time" queue in the event manager,
  causing the third argument of Nasal's settimer() (a flag for "sim
  time") to be ignored.  Inverts the default sense of the argument,
  as there are lots of uses of settimer() in the current code,
  almost none of which want to use real time.

  Note this fix introduces a header file incompatibility in SimGear
  -- be sure to update.

2005-11-09 19:47  curt

* NEWS: v0.9.9-pre2 changes.

2005-11-09 19:41  curt

* configure.ac: v0.9.9-pre2 changes (just the version number!)

2005-11-05 21:32  curt

* NEWS: Spelling fixes and other small corrections.

2005-11-05 20:30  curt

* Doxyfile, Makefile.am, NEWS, SimGear.dsp, configure.ac: Some
  pre-release updates.

2005-11-05 19:47  curt

* simgear/sound/xmlsound.cxx: Add a code comment for future
  thought.

2005-11-01 10:45  ehofman

* simgear/screen/: ssgEntityArray.cxx, ssgEntityArray.hxx: Move
  Curt's ssgEntityArray experiment over to SimGear.

2005-10-30 16:05  ehofman

* configure.ac: Remove some unused code.

2005-10-27 10:21  ehofman

* simgear/scene/: material/matmodel.cxx, model/modellib.cxx:
  Mathias Fr�hlich:

  I guess the most important memory leaks are plugged now.  Just by
  inspection: An other memory leak in Simgear.

2005-10-27 10:21  ehofman

* simgear/constants.h: MSVC fix.

2005-10-26 13:19  ehofman

* simgear/threads/SGThread.hxx: Back out the shared mutex code
  since it only works when the mutex is in shared memory[1],
  something we don't support anyhow.  This also fixes a FreeBSD
  compile problem.

  [1]
  http://hypermail.linklord.com/new-httpd.old/2002/Jan/0557.html

2005-10-25 20:05  ehofman

* simgear/sound/: sample_openal.cxx, soundmgr_openal.cxx: Oops,
  ALUT 1.0 requires a little more work than expected.

2005-10-25 15:48  ehofman

* simgear/: bucket/newbucket.cxx, environment/metar.hxx,
  ephemeris/moonpos.cxx, math/fastmath.cxx, math/point3d.hxx,
  math/polar3d.cxx, math/polar3d.hxx, misc/texcoord.cxx,
  route/waypoint.hxx, scene/sky/sky.cxx, scene/sky/sphere.cxx,
  scene/sky/stars.cxx, threads/SGThread.hxx, timing/sg_time.cxx:
  Alex Romosan:

  * Use "const string&" rather than "string" in function calls when
  appropriate.	* Use "const Point3D&" instead of "Pint3D" in
  function calls when appropriate.  * Improved course calculation
  in calc_gc_course_dist() * Safer thread handling code.

  Vassilii Khachaturov:

  Dont use "const Point3D&" for return types unless you're
  absolutely sure.

  Erik Hofman:

  * Use SGD_(2)PI(_[24]) as defined in simgear/constants.h rather
  than	 calculating it by hand every time.

2005-10-25 15:06  ehofman

* simgear/sound/: openal_test1.cxx, sample_openal.cxx: Prepare for
  ALUT version 1.0

2005-10-23 16:04  ehofman

* simgear/: props/props.cxx, scene/material/mat.cxx: Cosmetic
  updates.

2005-10-23 15:47  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx, matlib.cxx,
  matlib.hxx: Slightly update the seasonal texture support code.

2005-10-23 15:31  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx, matlib.cxx,
  matlib.hxx: Add support for seasons.

2005-10-23 13:55  ehofman

* simgear/props/: props.cxx, props.hxx: Melchior FRANZ:

  The attached patch makes remove_child() available as
  removeChild(pos, keep).  That's consistent with getChild. Only
  renamed remove_child to removeChild and added a check for
  validity of the pos argument.

  removeChildren() will be used in input.cxx, and a lot in the
  upcoming dynamic new_gui dialogs, which are aiming at replacing
  the hard-coded dialogs. I'll discuss them on the list once the
  infrastructure is in place. (The <hide> gui property was one part
  of it.)

2005-10-23 13:46  ehofman

* simgear/scene/sky/: dome.cxx, dome.hxx: Harald JOHSEN: the sky
  color now fades to black with altitude

2005-10-22 13:07  ehofman

* simgear/Makefile.am: Remove some dead code.

2005-10-16 19:23  ehofman

* simgear/scene/model/shadanim.cxx: Harald JOHNSEN:

  I have corrected a few bugs with the owner draw gauge, weather
  radar code and heat-haze effect.

  - shadanim.cxx :   the heat/haze effect was showing artifacts
  when using a screen resolution >   1024 pixels.

2005-10-15 16:59  ehofman

* simgear/screen/shader.cpp: Ima Sudonim remembered me there is
  still one problem for gcc Vs. 3.4 or later under Cygwin. This
  fixes it.

2005-10-14 18:42  andy

* simgear/nasal/gc.c: Fix memory leak discovered by Mathias
  Froehlich

2005-10-14 18:27  ehofman

* simgear/props/props.hxx: Mathias Fr�hlich:

  This one, removes some virtual qualifiers at a private member
  class of SGPropertyNode. These virtual qualifiers are really
  useless and stop the compiler from inlineing these functions. I
  gain a single frame with my favourite aircraft per second!

2005-10-14 18:21  ehofman

* simgear/: props/props.cxx, screen/texture.cxx,
  screen/texture.hxx: Mathias Fr�hlich:

  I have done a valgrind run in flightgear. Just start it up and
  close it at the fist change I had about half an hour later.

  property-leak.diff:	 A leak in the property system which I
  could even notice in top.

  texture-leak.diff:	 minor one, but fixed is fixed ...

2005-10-12 18:43  curt

* simgear/io/sg_binobj.cxx: Use an unsigned vs. signed short to
  double our element capacity for higher resolution scenery.

2005-10-12 10:59  ehofman

* simgear/screen/extensions.hxx: Martin Spott:

  make GCC on Solaris8 happy.

2005-10-11 20:56  curt

* configure.ac: Fix a small typo.

2005-10-09 11:37  ehofman

* simgear/: ephemeris/ephemeris.cxx, ephemeris/stardata.cxx,
  io/decode_binobj.cxx, io/sg_socket_udp.cxx, scene/sky/moon.cxx:
  David Luff:

  The following patch needs to be applied to fix the errors that
  Georg Vollnhals was getting whilst attempting to compile SimGear
  with gcc-3.4.x

2005-10-08 13:52  ehofman

* simgear/scene/model/: animation.hxx, shadanim.cxx: Harald
  JOHNSEN:

  - shadanim.cxx, animation.hxx :   new chrome (sphere mapping)
  shading ;   disabled the loading of the fresnel VP until it is
  fixed ;

2005-10-06 16:39  ehofman

* simgear/misc/stdint.hxx: Another Solaris fix.

2005-10-06 13:06  ehofman

* simgear/: compiler.h, timing/sg_time.cxx: Martin Spott: Use
  standardized Sun directive.

2005-10-06 11:45  ehofman

* simgear/environment/metar.cxx: MSYS fix.

2005-10-06 10:25  ehofman

* simgear/: io/sg_file.cxx, misc/stdint.hxx: MSVC fixes. Frederic:
  MSVC has no ssize_t type

2005-10-01 13:41  ehofman

* simgear/io/: lowlevel.cxx, lowtest.cxx, sg_binobj.cxx: Cygwin
  fixes(?), it's a good idea to do it this way anyhow.

2005-09-29 14:05  ehofman

* simgear/misc/swap_test.cpp: Cygwin fixes.

2005-09-28 10:03  ehofman

* simgear/scene/model/animation.cxx: Fix an oops.

2005-09-28 10:00  ehofman

* simgear/scene/model/animation.cxx: Back out a patch from Sept.
  25th. Setting the *factor* to 0.0 by default isn't generally a
  good idea.

2005-09-26 23:01  curt

* simgear/serial/serial.cxx: Make some adjustment to low level
  serial port configuration flags for unix.

2005-09-25 09:44  ehofman

* simgear/scene/model/: Makefile.am, animation.cxx, animation.hxx,
  model.cxx, shadanim.cxx: Vivian Meazza:

  Correct the bug in the translate animation where the offset was
  part of the multiplication. It now behaves like all other
  animations: out = (prop * factor) + offset

  I feel strongly that the existing is wrong and must be corrected
  - it is non-op if the offset is zero as I have found to my cost!
  It is just a typo I expect.

  The diff also provides non-op default values for the scale
  animation.

  I've also included Harald's latest eye-candy animation which
  allows us to have a very smart heat-haze for exhausts. They have
  been tested by me and others on Linux and Cygwin. You might like
  to upload these - I have a revised Hunter ready to go as soon as
  they are uploaded.

2005-09-24 14:28  curt

* simgear/io/: iochannel.cxx, iochannel.hxx, sg_file.cxx: Add eof()
  support to SGIOChannel/SGFile.

2005-09-23 22:13  curt

* simgear/io/: sg_file.cxx, sg_file.hxx: Add an eof() method to
  SGFile.

2005-09-23 14:30  ehofman

* simgear/misc/stdint.hxx: AMD64 and sgi fixes.

2005-09-22 15:43  ehofman

* simgear/misc/swap_test.cpp: Platform compatibility fix.

2005-09-22 11:15  ehofman

* simgear/misc/stdint.hxx: Add some linefeeds.

2005-09-22 11:11  ehofman

* simgear/: compiler.h, io/lowlevel.hxx, io/lowtest.cxx,
  misc/Makefile.am, misc/stdint.hxx, misc/swap_test.cpp: Create our
  own stdint.h(xx) implementation and use it where needed.

2005-09-21 11:22  ehofman

* configure.ac: Prepare for Openal 1.1 and a separate alut library

2005-09-20 23:38  andy

* simgear/nasal/misc.c: Oops, Frederic caught an inline declaration
  that had snuck into the code.

2005-09-20 23:09  andy

* simgear/nasal/: code.c, code.h, codegen.c, data.h, gc.c, hash.c,
  lex.c, lib.c, mathlib.c, misc.c, nasal.h, parse.c, string.c,
  thread-posix.c, thread-win32.c, vector.c: Sneak a Nasal update in
  before the next release.  This version *appears* to work
  correctly on all systems to which I have access (i386
  linux/win32, x86_64 linux, powerpc OS X, Sparc Solaris 10), but
  not all systems are capable of running fgfs.	Beyond that,
  multiple threading bugs were fixed, and the naCall() API changed
  slightly to support named function arguments.

  NOTE: this introduces a change in the external API, and therefore
  this change *must* be compiled against current FlightGear code.

2005-09-18 23:05  ehofman

* simgear/io/lowlevel.hxx: Don't refference simgear_config.h
  because this header gets installed :-(

2005-09-18 11:21  ehofman

* simgear/io/lowlevel.hxx: int64_t is in stdint.h by default.

2005-09-18 11:19  ehofman

* simgear/io/lowlevel.hxx: MSVC fix.

2005-09-15 19:06  ehofman

* simgear/io/: lowlevel.cxx, lowlevel.hxx, sg_binobj.cxx: Use
  inttypes.h specified types. This is the standard and fixes some
  64-bit problems.

2005-09-15 18:54  ehofman

* simgear/xml/easyxml.cxx: Better XML error catching, proposed by
  Richard Harrison.

2005-09-05 15:30  ehofman

* simgear/timing/timestamp.cxx: Vivian Meazza:

  After much trial and tribulation, Harald came up with a fix for
  the bug which has been plaguing Cygwin for a couple of weeks now.

  It's only a couple of lines. I've tested it exhaustively, and it
  seems to cure the problem of Cygwin failing to start.

2005-09-05 15:23  ehofman

* simgear/scene/model/: location.cxx, location.hxx, placement.cxx,
  placement.hxx: Mathias Fr�hlich:

  There was a patch from Manuel Masing a few months ago which
  cleaned up SGLocation's way depending on input values. That means
  that with that patch SGLocation does no longer have calls with
  unneeded input arguments.  I took his patch and integrated that
  into flightgear and made maximum use of that changes.

2005-09-05 11:02  ehofman

* simgear/screen/: RenderTexture.cpp, extensions.cxx: Mathias
  Fr�hlich:

  just a few split out patches from my zoo of local work ...

  The patch to simgear-glxproc.diff changes dlopen to not open a
  specific library.  If it is used with a NULL argument, we just
  get a handle to the current running binary including all loaded
  libraries. This has the advantage that we do not rely on the name
  of libGL on the specific platform.  Also a user can link with his
  own different named libGL or with a static libGL.a

  Then the render texture again ...

  glxQueryVersion turns out to return the  minimum of the client
  libraries glx version and the servers glx version. *All* Xorg
  servers return 1.2 here.  So we never get the glxPBuffer
  functions  which are the only ones working with ati's drivers ...
    Reverted back to checking the required functions and just use
  them if they are there. Still prefering the glx standard variants
  since they work on ati's drivers ...

2005-09-05 10:22  ehofman

* simgear/compiler.h: Add some more defines as specified in
  FlightGear/src/Network/net_fdm_mini.hxx

2005-09-05 10:17  ehofman

* simgear/: compiler.h, scene/model/shadowvolume.cxx,
  scene/sky/bbcache.cxx, scene/sky/cloud.cxx,
  scene/sky/cloudfield.cxx, screen/shader.cpp: Mac OS X fixes from
  Markus Morawitz stdint.h replacement defines for Windows and Sun
  from Frederic et all.

2005-08-22 19:44  ehofman

* simgear/scene/: model/model.cxx, sky/newcloud.cxx: Harald
  JOHNSEN:

  - model.cxx :   load the 2.5D panels before the animations so
  that the panels can be used in   animations his solve the problem
  of 2.5D panels visible outside of the   aircraft (one can add a
  null animation to put the panel at the top of the   aircraft
  graph so it is drawn first) and this adds the possibility to have
    billboarded/popup panels.

  - newcloud.cxx :   removed 'this' pointer cast for amd64
  compiler.

2005-08-10 10:04  ehofman

* simgear/timing/timestamp.cxx: Cygwin fix.

2005-07-31 10:56  ehofman

* simgear/scene/sky/: cloudfield.cxx, cloudfield.hxx: Harald
  JOHNSEN:

  added a cull test on fields

2005-07-31 10:46  ehofman

* simgear/screen/extensions.hxx: Fix a problem with systems that
  don't define GLXPbufferSGIX or GLXFBConfigSGIX

2005-07-31 09:59  ehofman

* simgear/screen/: Makefile.am, extensions.hxx, shader.cpp,
  shader.h: Harald JOHNSEN:

  This is the low level shader class for Simgear.  It's the code
  from Roman Grigoriev with a few adaptations.

2005-07-27 10:02  ehofman

* simgear/: scene/sky/cloud.cxx, sound/soundmgr_openal.cxx: MacOS-X
  fixes.

2005-07-22 01:03  andy

* simgear/nasal/string.c: Josh discovered a bug parsing negative
  numbers with leading zeros ("-0.3") which also affected ones of
  the form "-.3".  This got introduced a few months back, I'm not
  sure how it went undetected for so long...

2005-07-18 18:57  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx,
  shadowvolume.cxx, shadowvolume.hxx: Harald JOHSEN:

  Changes =======

  - shadowvolume.cxx, renderer.cxx :   - reduced the polygon offset
  a bit to eliminate some artifact ;   - changed again the cleanup
  code for objects inside a tile because it could crash on rare
  occasion ;   - the culling of shadow casters has been rewritten
  to traverse the scene graph, it should be	a bit faster when
  there is a lot of objects ;	- the range selector was not
  correctly handled, sometimes the wrong LOD was casting shadows.
    - added the option to display aircraft's transparent objects
  after the shadows, this will	    reduce the problem of shadows
  being hidden by the transparent object (propeller disk,
  rotor, etc). A side effect is that aircraft's transparent objects
  won't receive shadows      anymore. This is usually a good thing
  except when the aircraft use a 'transparent'	    texture where
  it should not. A transparent texture in the plib context is a
  texture      with an alpha channel or a material with alpha <=
  0.99.

  - model.cxx, animation.cxx, shadowvolume.cxx :   - added an
  optional <condition> under the <noshadow> animation

  - tower.cxx	- correct a rare bug where all occurences of the
  aircraft are not deleted from the   departure list causing a
  crash in FGTower::CheckDepartureList function.

2005-07-13 14:00  ehofman

* simgear/screen/: RenderTexture.cpp, RenderTexture.h,
  extensions.cxx, extensions.hxx: Adjustments to better support
  GLX1.3 and ATI drivers.

2005-07-06 10:44  ehofman

* simgear/scene/model/: shadowvolume.cxx, shadowvolume.hxx: Harald
  JOHNSEN:

  Melchior has found another bug, I tried to skip some computation
  for a few frames but that introduced some bad rendering bug with
  the aircraft moving parts.  I corrected that and reduced a bit
  the cpu usage for ground objects.

2005-07-05 20:53  ehofman

* simgear/scene/model/shadowvolume.cxx: Another update, the
  previous one could crash if you leave the surrounding tiles (try
  Set aircraft in air and choose a distant airport).

2005-07-05 20:00  ehofman

* simgear/scene/model/shadowvolume.cxx: Somehow gcc allows function
  overriding but MIPSpro doesn't. Fix this.

2005-07-05 19:08  ehofman

* simgear/: scene/model/animation.cxx, scene/model/animation.hxx,
  scene/model/model.cxx, scene/model/model.hxx,
  scene/model/shadowvolume.cxx, scene/model/shadowvolume.hxx,
  scene/sky/bbcache.cxx, screen/RenderTexture.cpp:
  Harald JOHNSEN:

  - shadow volume vertex are now shared, using DrawElements instead
  of repeated	calls to glVertex, this can improve performance on
  some systems.  - added a rendering path that use the alpha
  channel instead of the stencill   buffer.  - releasing memory
  when tiles objects are destroyed - objects sub parts will not
  cast shadows if their name begins with "noshadow"   or if they
  are in a <noshadow> animation

  - bbcache.cxx :    don't ask for a 32 bits context when the
  primary context is only 16 bits

  - RenderTexture.cpp :    corrected a crash when asking for a
  second rendering context    on win32 and extensions not being
  supported

  - model.cxx, animation.cxx :	  added a <noshadow> animation,
  added an animation type needed by the shadow	  code.

2005-07-04 11:20  ehofman

* simgear/sound/openal_test1.cxx: Make sure it works with the lates
  version of OpenAL.

2005-06-30 21:10  ehofman

* simgear/props/: props.cxx, props.hxx: Restore the old behavior.
  Additions are likely.

2005-06-29 11:41  ehofman

* simgear/props/: props.cxx, props.hxx: Melchior FRANZ:

  - check for isTied() and refcount has to be made *before* we go
  into	 recursion, so as to pertain subtrees of refcounted nodes,
  even if there   are no refcounted/tied nodes *in* this tree -
  return value inverted, because it's more logical to say
  removeChildren() == true --> everything removed;  false -->
  failed - further cleanup

2005-06-28 13:19  ehofman

* simgear/props/props_io.cxx: Frederic: Also copy the attributes
  over to the new tree.

2005-06-28 13:19  ehofman

* simgear/props/: props.cxx, props.hxx: Due to a misunderstanding
  of what removeChild() actually does, some used it to detach a
  subtree from the main tree. The previous patch broke that
  behaviour so a new function call detchChild() is now added.

2005-06-27 19:48  ehofman

* simgear/props/props.cxx: fix return value

2005-06-27 15:49  ehofman

* simgear/props/: props.cxx, props.hxx: Melchior FRANZ:

  - introduce removeChildren() and removeChildren(name)  to remove
  all children	 or all with a given name - let removeChild() and
  removeChildren() also remove child trees, and let them   return a
  "dirty" boolean that indicates if one or more subnodes had to be
    kept because of refcounting (removeChild returned a
  SGPropertyNode_ptr before) - make alias/unalias increase/decrease
  the refcounter - don't remove refcounted or tied nodes

  This patch makes the SGPropertyNode_ptr actually useful. Until
  today, they did proper refcounting (except for aliases), but no
  other part did check this counter.

  But SGPropertyNode_ptr aren't only useful for the first time,
  they are now highly recommended for every place that relies on a
  node address, and wants to "lock" it (so that removeChild(ren)
  will never try to remove them). This is not guaranteed for
  SGPropertyNode* (and never was). Of course, that's not an
  imminent problem, as only four places currently use
  removeChild(ren) and these are careful to only remove their own
  data.

2005-06-26 19:16  ehofman

* simgear/: environment/visual_enviro.hxx, scene/model/Makefile.am,
  scene/model/shadowvolume.cxx, scene/model/shadowvolume.hxx:
  Harald JOHNSEN:

  Changes =======

  New volumetric shadows for FlightGear.

  There is now two new checkboxes in the rendering dialog to
  enable/disable shadows for the user aircraft and for static
  scenery objects (ie those defined in the .stg files).  AI and
  random objects are not handled for the moment.

  known bugs ========== - ghost objects

2005-06-25 13:22  ehofman

* configure.ac, simgear/scene/sky/Makefile.am: Remove the 'old' 3D
  clouds code.

2005-06-12 13:23  ehofman

* simgear/scene/model/animation.cxx: Melchior: Make the (lack of)
  axis or center location definitions more consistent.

2005-06-11 10:39  ehofman

* simgear/structure/: subsystem_mgr.cxx, subsystem_mgr.hxx:
  Melchior FRANZ:

  This is the more elegant solution that Andy had proposed in a
  response to my RFC on Nasal initialization code in joystick
  configuration files.	As Nasal is initialized last (for good
  reason), subsystem can currently not use it for initializing.
  postinit() is called on all subsystems after all have been
  initialized.

2005-06-08 16:07  ehofman

* simgear/props/props.cxx: fix a coredump situation, discovered by
  Melchior.

2005-05-30 11:04  ehofman

* simgear/: environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, scene/sky/cloudfield.cxx,
  scene/sky/newcloud.cxx, scene/sky/newcloud.hxx: Harald JOHSEN:

  Changes =======

  - changed the rotation of sprites, they don't rotate strangely
  when we   approach them now - corrected the strange movement of
  clouds when banking quickly - it no more rain above cloud layers
  - add a radar echo container used by the weather radar instrument

2005-05-29 18:13  andy

* simgear/nasal/: code.c, lib.c: Fix two crash conditions Ampere
  found.  These are just temporary patches; my private version has
  rewritten both of these functions (ironically fixing these bugs
  in the process) to handle negative offsets meaning "from the
  end".

2005-05-24 10:13  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx: Melchior
  FRANZ:

  Turn the material animation's <transparency> property into a
  group, with members <alpha-prop>/<alpha>, <offset-prop>/<offset>,
  <factor-prop>/<factor>, <min>, and <max>. The "material"
  animation can now fully replace "blend" and "alpha-test" (-->
  <threshold>) animations, with enhanced possibilities: The
  "material" animation can be used for one or more explicit objects
  (like "blend"), but also for all objects that share one material
  (<global>), which avoids problems with objects being forced
  together into one tree. Also, an object doesn't have to be
  semitransparent or textured with a semitransparent texture to
  make blending work. Unlike the "blend" animation, the "material"
  animation also makes fully opaque and untextured objects
  transparent. (This fixes the bo105's formerly semi-transparent
  rotor.)

  Erik: The blend animation and alpha-test animation are
  depreciated as of now.

2005-05-23 18:35  ehofman

* simgear/scene/model/animation.cxx: Melchior FRANZ:

  Currently, the material animation sets
  glColorMaterial(GL_AMBIENT_AND_DIFFUSE) for all material
  properties. This breaks emission-only (e.g. cockpit lighting for
  the p51d) or specular-only animation. ==> set glColorMaterial
  only where it is really required.

2005-05-22 11:18  ehofman

* simgear/misc/strutils.cxx: MSVC fix.

2005-05-22 10:09  ehofman

* simgear/: environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, scene/sky/bbcache.cxx,
  scene/sky/cloudfield.cxx, scene/sky/cloudfield.hxx: Harald
  JOHSNEN:

  Changes =======

  - correct the transparency probleme when old 3d clouds were
  enabled  (rendering context with an alpha channel) - changed rain
  cone orientation, it can now be viewed from helicopter or chase
  view (still not tower view) - clouds are a bit more yellow/red at
  dawn/dusk - weather data is now correctly propagated to the
  interpolator, this correct   visibility, wind, etc - the 'metar'
  weather scenario now immedialty reuse the real metar data - real
  metar no more overwrite custom weather scenario

2005-05-22 09:35  ehofman

* simgear/environment/visual_enviro.hxx: MSVC fix.

2005-05-17 11:56  ehofman

* simgear/props/: props.cxx, props.hxx: Make removeChild() work
  (again?)

2005-05-15 11:34  ehofman

* simgear/environment/visual_enviro.hxx: gcc fix.

2005-05-15 11:27  ehofman

* simgear/: environment/visual_enviro.cxx,
  environment/visual_enviro.hxx, scene/sky/bbcache.cxx,
  scene/sky/bbcache.hxx, scene/sky/cloud.cxx, scene/sky/cloud.hxx,
  scene/sky/cloudfield.cxx, scene/sky/cloudfield.hxx,
  scene/sky/newcloud.cxx, scene/sky/newcloud.hxx,
  scene/sky/sky.cxx, screen/Makefile.am: Harald JOHNSEN:

  This is another update for the cloud code, a lot of lines but
  this time I have started to add the doxygen doc.

  Misc ====

  - corrected a bug when RTT is not available, the current
  rendering context was   altered - if RTT is not available then 3d
  clouds are not drawn at all - impostors lighting is now
  recomputed when the sun changes position - distant objects are no
  more seen in front of clouds - blending of distant clouds is a
  bit better now - litle optimization of code (uses a less cpu
  time) - use layer wind speed and direction (no more hardcoded
  wind) - fov is no more hardcoded

  Changes =======

  - clouds (cu only) are dissipating/reforming (experimental) -
  compute a turbulence factor that depends on surrounding clouds
  and type of	clouds (experimental) - clouds shapes are defined
  in cloudlayers.xml - type of clouds present in a layer is also
  defined in cloudlayers.xml - cloud layers are generated from
  metar and other misc. data (in progress) - added a rain effect
  around the viewer (enabled in the rendering dialog and   when the
  metar property says so) - added a lightning effect (enabled in
  the rendering dialog) : cb clouds spawn   new lightnings - added
  a dialog to select from different weather source :
  metar/property,   a 'fair weather' environment and a
  'thunderstorm' environment.

2005-05-09 18:18  ehofman

* simgear/props/props_io.cxx: Melchior: Only change types when
  explicitly requested.

2005-05-09 16:31  ehofman

* simgear/props/: props.cxx, props.hxx, props_io.cxx: Melchior
  FRANZ:

  Vivian pointed out that a redefined Ctrl-U key binding didn't
  work correctly. I found out that this is, because the definition
  in $FG_ROOT/keyboard.xml sets <value type="bool"> for binding[1],
  and ... [better sit down first!] ... and assigning <value
  type="double"> in a *-set.xml file doesn't *really* set "double"
  as new type!

  Instead, the boolean is kept, and a double sqeezed into it. In
  other words: once tainted as bool, you can throw all doubles in
  the universe on a property node, and all it will accept is 0 and
  1. Without warning!

  BTW: I changed the patch: I was overly cautious: clear_value()
  does already care for ties and for setting NONE, so we just need
  to make that public as clearValue(), and use that. Makes the
  patch a bit more verbose, though.  :-/

2005-05-07 10:46  ehofman

* simgear/scene/sky/newcloud.cxx: Solaris fix.

2005-05-04 22:17  andy

* simgear/nasal/code.c: Properly release the mod lock when
  returning from a runtime error.  Ampere discovered that the
  interpreter would deadlock at runtime if it hit such a condition
  during initialization.

2005-05-01 10:50  ehofman

* simgear/bucket/newbucket.cxx: Phil Cazzola:

  This is a minor bug fix for sgBucketDiff().  If you crossed the
  bucket size boundary, the answer for dx could be wrong.

  E.g.	 going from   0:0, 21:7  to 0:7, 21:7	would give you dx =
  7 (correct)	but going from 0:0, 21:7 to 0:3, 22:0 would give
  you dx = 6 (instead of 7)

  Previously it differenced the center longitudes of the buckets.
  When you cross a boundary, the center point of the larger bucket
  now lies on the edge of the smaller bucket.

  The result was a dx with an integer + 1/2 bucket, which rint()
  was rounding to the nearest even int.

  This function only seems to be used in TerraGear.

2005-04-30 12:00  ehofman

* simgear/scene/sky/: cloudfield.cxx, newcloud.cxx: Make use of the
  repeatable sg_random() function so display systems can
  synchronize 3d clouds too.

2005-04-30 11:59  ehofman

* simgear/math/: sg_random.c, sg_random.h: Add a seed function that
  gives the same random seed within a ten minute period of time.
  This should be useful for synchronizing display systems.

2005-04-29 16:37  ehofman

* simgear/scene/model/: placementtrans.cxx, placementtrans.hxx:
  Mathias:

  I have done a patch to eliminate the jitter of 3D-objects near
  the viewpoint (for example 3D cockpit objects).  The problem is
  the roundoff accuracy of the float values used in the scenegraph
  together with the transforms of the eyepoint relative to the
  scenery center.

  The solution will be to move the scenery center near the view
  point.  This way floats relative accuracy is enough to show a
  stable picture.

  To get that right I have introduced a transform node for the
  scenegraph which is responsible for that shift and uses double
  values as long as possible.  The scenery subsystem now has a list
  of all those transforms required to place objects in the world
  and will tell all those transforms that the scenery center has
  changed when the set_scenery_center() of the scenery subsystem is
  called.  The problem was not solvable by SGModelPlacement and
  SGLocation, since not all objects, especially the scenery, are
  placed using these classes.

  The first approach was to have the scenery center exactly at the
  eyepoint.  This works well for the cockpit.  But then the ground
  jitters a bit below the aircraft. With our default views you
  can't see that, but that F-18 has a camera view below the left
  engine intake with the nose gear and the ground in its field of
  view, here I could see that.	Having the scenery center constant
  will still have this roundoff problems, but like it is now too,
  the roundoff error here is exactly the same in each frame, so you
  will not notice any jitter.

  The real solution is now to keep the scenery center constant as
  long as it is in a ball of 30m radius around the view point. If
  the scenery center is outside this ball, just put it at the view
  point.

  As a sideeffect of now beeing able to switch the scenery center
  in the whole scenegraph with one function call, I was able to
  remove a one half of a problem when switching views, where the
  scenery center was far off for one or two frames past switching
  from one view to the next. Also included is a fix to the other
  half of this problem, where the view position was not yet copied
  into a view when it is switched (at least under glut). This was
  responsible for the 'Error: ...' messages of the cloud subsystem
  when views were switched.

2005-04-29 16:36  ehofman

* simgear/scene/model/: Makefile.am, location.hxx, placement.cxx,
  placement.hxx: Mathias:

    have done a patch to eliminate the jitter of 3D-objects near the
  viewpoint
  (for example 3D cockpit objects).  The problem is the roundoff
  accuracy of the float values used in the scenegraph together with
  the transforms of the eyepoint relative to the scenery center.

  The solution will be to move the scenery center near the view
  point.  This way floats relative accuracy is enough to show a
  stable picture.

  To get that right I have introduced a transform node for the
  scenegraph which is responsible for that shift and uses double
  values as long as possible.  The scenery subsystem now has a list
  of all those transforms required to place objects in the world
  and will tell all those transforms that the scenery center has
  changed when the set_scenery_center() of the scenery subsystem is
  called.  The problem was not solvable by SGModelPlacement and
  SGLocation, since not all objects, especially the scenery, are
  placed using these classes.

  The first approach was to have the scenery center exactly at the
  eyepoint.  This works well for the cockpit.  But then the ground
  jitters a bit below the aircraft. With our default views you
  can't see that, but that F-18 has a camera view below the left
  engine intake with the nose gear and the ground in its field of
  view, here I could see that.	Having the scenery center constant
  will still have this roundoff problems, but like it is now too,
  the roundoff error here is exactly the same in each frame, so you
  will not notice any jitter.

  The real solution is now to keep the scenery center constant as
  long as it is in a ball of 30m radius around the view point. If
  the scenery center is outside this ball, just put it at the view
  point.

  As a sideeffect of now beeing able to switch the scenery center
  in the whole scenegraph with one function call, I was able to
  remove a one half of a problem when switching views, where the
  scenery center was far off for one or two frames past switching
  from one view to the next. Also included is a fix to the other
  half of this problem, where the view position was not yet copied
  into a view when it is switched (at least under glut). This was
  responsible for the 'Error: ...' messages of the cloud subsystem
  when views were switched.

2005-04-26 22:14  ehofman

* simgear/scene/sky/cloudfield.cxx: Harald Johnsen: Fix a
  'terrible' bug with culling of the clouds.

2005-04-26 11:08  ehofman

* simgear/scene/sky/: cloudfield.cxx, newcloud.cxx: IRIX fixes.

2005-04-26 10:30  ehofman

* simgear/: scene/sky/bbcache.cxx, scene/sky/cloudfield.cxx,
  scene/sky/cloudfield.hxx, scene/sky/newcloud.cxx,
  environment/visual_enviro.cxx, environment/visual_enviro.hxx:
  Harald Johnson:

  Changes =======

  - corrected some strange behavior when playing with the render
  dialog options - the density slider is now working : if you are
  fps limited and still want to see clouds in	the distance you
  should play with that - added the choice for texture resolution,
  its more comprehensible now (before it was   wrongly allways
  choosing 64x64 textures) - changed the initial texture size : you
  now have 64 texture of 64x64, this uses 1Mo of   texture memory
  (before it was 20 texture of	256x256, that took more memory and
  there was   not enought impostors) - sun vector is now right so
  the lighting is a bit better - removed useless sort and light
  computations for impostors, this should save a lot of cpu -
  blending of distant cloud is more accurate now - clouds are now
  positioned correctly, they don't try to escape you anymore - no
  more red/white boxes around cloud - textures are now filtered (no
  more big pixels)

  known bugs ==========

  - distant objects are seen in front of clouds

2005-04-24 15:55  ehofman

* simgear/scene/sky/: bbcache.cxx, cloudfield.cxx, newcloud.cxx:
  Don't refference GLUT but GLU instead.

2005-04-24 13:45  ehofman

* simgear/environment/: visual_enviro.cxx~, visual_enviro.hxx~:
  This one time I did a commit using Linux. <sigh>

2005-04-24 13:16  ehofman

* simgear/: environment/Makefile.am, environment/visual_enviro.cxx,
  environment/visual_enviro.cxx~, environment/visual_enviro.hxx,
  environment/visual_enviro.hxx~, scene/sky/Makefile.am,
  scene/sky/bbcache.cxx, scene/sky/bbcache.hxx,
  scene/sky/cloud.cxx, scene/sky/cloud.hxx,
  scene/sky/cloudfield.cxx, scene/sky/cloudfield.hxx,
  scene/sky/newcloud.cxx, scene/sky/newcloud.hxx:
  Harald Johnson:

  - new and updated sources for the new volumetric clouds - 2 new
  textures for the clouds - an update to the render dialog to
  enable/disable and change a few parameters   for the new clouds

2005-04-22 23:54  andy

* simgear/nasal/: code.c, code.h, codegen.c, lex.c, lib.c, parse.c,
  parse.h: Support for a "forindex(idx; list) {...}" construct
  analagous to foreach, except that the variable gets the index
  instead of the list element.	Should be useful, and took almost
  no code to implement.

  Support for operator/assignment syntax: +=, -=, *=, /= and ~= now
  do what you think they should.

  Library support for a bind() function (see the docs Andy is still
  writing), allowing runtime modifications to function lexical
  environments.

2005-04-19 16:19  andy

* simgear/nasal/: hash.c, lib.c, parse.c: Fix clamping of the
  minimum hash size, because the Melchior discovered   that the
  column math goes wacky when lgalloced is allowed to be   zero.
  Augment the find() function to take a starting index.  Fix strc()
  to use a default index of zero.  Fix parser precedence of
  TOK_MINUS, so that "a-b-1" means (a-b)-1 and	 not a-(b-1).

2005-04-19 14:30  ehofman

* simgear/nasal/: code.c, data.h, lib.c, misc.c: Non gcc fixes.

2005-04-18 22:43  andy

* simgear/nasal/codegen.c: Fix crash in the code generator when
  compiling a (now illegal, because "var" is a reserved word)
  expresssion of the vorm "var=<expr>".

2005-04-18 21:48  andy

* simgear/nasal/: Makefile.am, code.c, code.h, codegen.c, data.h,
  debug.c, gc.c, hash.c, lex.c, lib.c, mathlib.c, misc.c, nasal.h,
  parse.c, parse.h, string.c, thread-posix.c, thread-win32.c,
  vector.c: Upgrade to nasal 1.0 test candidate

2005-03-30 20:45  andy

* simgear/nasal/code.c: Fix boolean semantics so that the empty
  string evaluates to false, and numeric strings are false if their
  numeric values are false.

2005-03-28 11:13  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx: Melchior
  FRANZ:

  Re-organisation: <diffuse>, <ambient>, <emission>, <specular> are
  now groups with members <red>, <green>, <blue>, <factor>,
  <offset>, and their <*-prop> forms. Additionally, there's an
  option <property-base> that can be used to set a path that is
  prepended to all <*-prop> paths.  It defaults to an empty string.
  Rationale: see model-howto.html.

2005-03-22 21:28  andy

* simgear/nasal/string.c: Don't parse a single "e" or "E" as a
  numerical zero.  You need a numerical prefix to use the 1.0e12
  notation, "e" alone is not enough.

2005-03-22 14:12  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx,
  model.hxx: Melchior FRANZ:

  here is the promised material animation. It looks a bit longish,
  but that wasn't avoidable -- there are simply too many parameters
  to consider. I tried hard, though, to make the animation fast by
  only doing the necessary stuff.  It doesn't affect the frame rate
  here with my test model. The animation is heavily based on Jim's
  "material-emission" animation.

  * implementation of the "material" animation (this required to
  make the   texture path available) + documentation update
  ($FG_ROOT/Docs/) * fix some more return values (texture
  animations, and select) for the   shadow problem (and some in
  anticipation of other problems  :-) * fix compiler warning

2005-03-19 11:19  ehofman

* simgear/scene/model/animation.cxx: Melchior FRANZ:

  the cause for the disappearing shadows is, that SimGear doesn't
  tell plib to call the pre-traversal-callback function on culled
  objects. These calls, however, are necessary to execute the
  transform animation that does, for example, translate a shadow
  back into the frustum! Curretnly, the callback is only executed,
  and the shadow only magically pops up again, when the object
  enters the frustum because the view has changed significantly.

  The plib documentation does only talk about TRUE and FALSE for
  possible return values from the pre-traversal-callback. But
  src/ssgEntity.cxx reads like this:

      int ssgEntity::preTravTests ( int *test_needed, int which )
      ...
    int result = (*preTravCB)(this,which) ;

    if ( result == 0 ) return FALSE ;
    if ( result == 2 ) *test_needed = 0 ;
      ...

  So the return value needs to be 2 to bypass the cull test for
  pretraversal, and get the pretraversal in any case. I only
  changed the return values in four animations: scale, rotate,
  translate, and range, because these are the most likely to move
  an object out of the frustum. It's not necessary for
  blend/alpha/texture manipulation etc. Of course, this is a bit
  more work for plib, but the performance will probably not be
  affected, because:

  * these four animations are mainly used for the aircraft model
  (the spin   and billboard (trees!) animations are not affected)

  * the number of extra nodes to process is quite low

  * a part of the time spent for the extra nodes to be processed,
  was before   used for workarounds that are now not necessary any
  more

  I didn't observe a frame rate drop, at least.

2005-03-12 16:51  andy

* simgear/nasal/string.c: Oops, fixed the wrong test

2005-03-12 16:49  andy

* simgear/nasal/string.c: Off by one error when printing exact
  poweres of ten

2005-03-11 22:49  andy

* simgear/nasal/string.c: Fix an infinite loop (due to an overflow
  condition) when printing some very large numbers.

2005-03-11 21:39  andy

* simgear/nasal/: codegen.c, string.c: Fix the fixes.  Note that
  "." had the same problem as "+" and "-", and that we can still
  match non-identical constants if they are both strings with the
  same numerical value.

2005-03-11 20:07  andy

* simgear/nasal/: codegen.c, parse.h, string.c: Don't parse the
  strings "+" and "-" as numerical zeros.  Also fix the code
  generation of constant objects to use real identity and not Nasal
  equality, so (e.g.) the constants 1 (number) and "1.0" (string)
  do not get turned into the same object in the generated code.

2005-03-10 09:58  ehofman

* simgear/sound/soundmgr_openal.cxx: Ima Sudonim:

  I have (hopefully) generated a patch for a previously mentioned
  simgear  problem for a hang condition in mac os x.  Mentioned in
  <http://baron.flightgear.org/pipermail/flightgear-devel/2005-February/
  035004.html>

2005-02-15 19:13  ehofman

* acinclude.m4: automake 1.8+ fixes

2005-02-12 13:44  ehofman

* simgear/screen/: RenderTexture.cpp, RenderTexture.h, colors.hxx:
  More MacOS X fixes

2005-02-11 16:19  ehofman

* configure.ac: Fixes from Norman for users running Cugwin with the
  XServer package installed.

2005-02-11 16:07  ehofman

* simgear/screen/RenderTexture.h: MacOS X fix(?)

2005-02-01 11:35  ehofman

* simgear/screen/: RenderTexture.cpp, RenderTexture.h: Comment out
  GLX code for MacOS and (hopefully) add some MacOS AGL compattible
  code. More needs to be done though.

2005-01-31 19:29  ehofman

* simgear/compiler.h: MacOS doesn't have glx.h

2005-01-31 19:21  ehofman

* simgear/scene/model/animation.cxx: Jim Wilson:

  Fix a couple of loose ends and missed edits on the earlier patch.
    For the most part no change in functionality.

2005-01-31 19:07  ehofman

* simgear/screen/RenderTexture.cpp: Cygwin fixes

2005-01-29 12:44  ehofman

* simgear/screen/: RenderTexture.cpp, TestRenderTexture.cpp,
  extensions.cxx, extensions.hxx: Windows fixes.

2005-01-29 11:31  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Jim Wilson:

  This patch adds support to the model animation system for
  modifying emissive states on the fly so that it is possible to
  make "lights" appear to dimm.

  This is an example of a configuration entry which should explain
  how it is used:

    <animation>
    <type>material-emission</type>
    <object-name>Face</object-name>
    <property>/controls/lighting/instruments-norm</property>
    <emiss-red>1.0</emiss-red>
    <emiss-green>0.8</emiss-green>
    <emiss-blue>0.5</emiss-blue>
    </animation>

  Note the color entries are the emissive colors when the
  "property" value is 1.0.  They are useful for tinting the light.
    The "property" itself must be float or double and is clamped to
  values between 0 ~ 1.0 inclusively.	The "property" value is
  multiplied against the colors to get the actual material
  properties.  Thus property value 0.0 = darkest, and 1.0 =
  brightest.

2005-01-28 16:23  ehofman

* simgear/environment/metar.cxx: MSVC fix.

2005-01-28 16:21  ehofman

* simgear/nasal/code.c: Frederic Bouvier:

  code.c is C code ( according to the file extension ), so
  variables should be declared at the beginning of the function.

2005-01-28 16:15  ehofman

* simgear/screen/texture.cxx: use a proper delete[]

2005-01-28 10:32  ehofman

* simgear/screen/: RenderTexture.cpp, RenderTexture.h,
  extensions.hxx: MSVC fixes

2005-01-27 18:49  ehofman

* simgear/screen/: RenderTexture.cpp, extensions.hxx: Fix an NVIDIA
  problem.

2005-01-27 11:56  ehofman

* simgear/screen/TestRenderTexture.cpp: Add a RenderTexture test
  program.

2005-01-27 11:47  ehofman

* simgear/sound/soundmgr_openal.cxx: Melchior FRANZ:

  If alcOpenDevice( NULL ) is NULL, then context is never assigned
  a value, and it's pointless to ask for it in the next "if". But
  as the ALCcontext that context points to doesn't seem to be fully
  defined (OpenAL bug), valgrind still complains ...

  Erik Hofman: Extend this some further and define context=0
  otherwise and check for context != 0 before using it.

2005-01-27 11:42  ehofman

* simgear/screen/texture.cxx: Melchior FRANZ:

  Trying to find the bug in tower.cxx (that crashes fgfs quite
  frequently for me!), I'm playing with valgrind again. Until I'm
  in the ATC subsystem there will be some other bugs and nitpicking
  along the way.

  valgrind doesn't like that imgage->tmp is once allocated with new
  and once with new[], sometimes with malloc() (via map), and
  sometimes freed with delete (not delete[]!) and sometimes with
  free(). With simple types such as GLubyte this shouldn't really
  make a difference, but anyway.

  Also, I promised that I'd send patches for "if (foo) delete foo;"
  as I'm making other changes to concerned files. texture.cxx is
  one with a few occurrences thereof. (Remember: C++ explicitly
  allows to delete null-pointers, so this check is redundant, and
  hence not tolerated in other projects, such as KDE. Doesn't have
  to impress us, of course.  :-)

  Also, fixes 4 signed/unsigned warnings (gcc 3.3.4)

2005-01-27 11:39  ehofman

* simgear/screen/: Makefile.am, RenderTexture.cpp, RenderTexture.h:
  Add Mark Haris' RenderTexture class based on SimGear's extesion
  support files.

2005-01-25 23:37  andy

* simgear/nasal/code.c: Move error handling in setupFuncall above
  the stack frame creation.  The error properly belongs to the
  enclosing scope, not the called (non-)function.  This bug was
  fixed a few months back in my private tree, but Melchior just
  discovered that the new Concorde scripts tickle it.  I really
  need to re-synchronize SimGear with my own Nasal tree...

2005-01-25 19:33  ehofman

* simgear/: compiler.h, screen/extensions.hxx: Add a bunch of
  extensions in preparation of render-to-texture support.

2005-01-24 22:46  curt

* simgear/scene/model/animation.cxx: Frederic Bouvier:

  The Beaver triggered a problem ( uninitialized variable ). Here
  is the updated code.

2005-01-24 20:49  curt

* simgear/scene/model/: animation.cxx, animation.hxx: Frederic
  Bouvier:

  this is the animation code that do randomisation of the spin
  animation. The XML tags are modified to support the syntax below
  :

    <use-personality type="bool">true</use-personality>
    <factor>
      <random>
  <min>1.8</min>
  <max>2.2</max>
      </random>
    </factor>
    <starting-pos-deg>
      <random>
  <min>0</min>
  <max>360</max>
      </random>
    </starting-pos-deg>

  instead of usual :

    <factor>1.42</factor>
    <starting-deg-pos>42.0</starting-deg-pos>

2005-01-24 16:51  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: Erik Hofman

  1. Remove the dependency on alut  which (on certein platforms)
  might pose	some restrictuons on commercial use.

  2. Create a sound source just prior to playing the sound and
  destroy it	again when the sound has stopped. This should
  greatly reduce the	error reports from Windows users.

2005-01-20 10:28  ehofman

* simgear/environment/: metar.cxx, metar.hxx: Melchior FRANZ:

  The following patches to SimGear & FlightGear ...

  - create an FGMetar abstraction layer, whose purpose is:   *
  provide defaults for unset values   * interpolate/randomize data
  (GREATER_THAN)   * derive additional values (time, age, snow
  cover)   * consider minimum identifier (CAVOK, mil. color codes)
  - add rain/hail/snow/snowcover support on the METAR side - add
  max age of METAR data handling (currently set to - add support
  for an external METAR cache proxy server - add CAVOK handling -
  set missing year/month in regular METAR messages - fix a small
  bug in metar.cxx (wrong return value)

2005-01-18 15:34  curt

* NEWS, configure.ac: Ready for 0.3.8 release.

2005-01-17 22:48  curt

* configure.ac: Require plib-1.8.4

2005-01-16 09:52  ehofman

* simgear/screen/texture.cxx: Use the double precission pow()
  function to get Solaris compiling.

2005-01-15 15:24  ehofman

* simgear/compiler.h: Solaris fix

2005-01-15 15:18  ehofman

* simgear/sound/soundmgr_openal.cxx: MingW/MSYS fix

2005-01-15 12:57  ehofman

* simgear/screen/texture.cxx: Eliminate some compiler warnings
  about converting float to int.

2005-01-15 11:48  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Add a make_bumpmap and
  a make_maxcolorwindow function, modify the make_normalmap
  function to maximize the color window before proceding.

2005-01-14 16:52  ehofman

* simgear/screen/texture.cxx: Add support for contrast.

2005-01-14 15:27  ehofman

* simgear/screen/texture.cxx: little endian fixes.

2005-01-14 14:36  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Add a make_grayscale
  function and call it from make_normalmap automatically, removing
  the need to do it make_grayscale prior to calling make_normalmap.

2005-01-14 14:12  ehofman

* simgear/screen/texture.cxx: Fix a mistake.

2005-01-14 14:08  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Add a function to
  calculate the normalmap from a regular texture.

2005-01-14 11:12  ehofman

* simgear/screen/texture.cxx: RGBA textures can be made monochrome
  also

2005-01-14 11:09  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Fix a few bugs and add
  a make_monochrome() function

2005-01-13 19:35  ehofman

* simgear/screen/texture.cxx: Some small updates to the saving
  code.

2005-01-13 19:05  ehofman

* simgear/screen/texture.cxx: Fix a crash situation.

2005-01-13 15:47  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Update the code a bit
  more, add a function to retreive the last error string and add a
  function to write SGI texture fils.

2005-01-11 17:02  curt

* simgear/scene/sky/: cloud.cxx, sky.cxx: - Fix a couple oops's in
  cloud.cxx - In sky.cxx blend low density cloud layers
  (few/scattered) into nothing (but   don't touch visibility
  distance) as we approach them so we can fly through	clean.	-
  For high density cloud layers (broken/overcast) we do not fade
  the layers   out, but we fade visibility to nearly nothing as we
  approach the layer.

2005-01-11 16:21  curt

* simgear/scene/sky/: cloud.cxx, cloud.hxx, sky.cxx: Add a method
  to SGCloudLayer to set overall cloud alpha.  This gives us the
  capability to slowly fade a cloud layer in or out.

  We use this effect in combination with lowering visibility as we
  approach a cloud layer to hide the fact that it is simply a 2d
  textured polygon being drawn across the sky.

2005-01-11 00:34  david

* simgear/scene/sky/sky.cxx: Do not reduce visibility when passing
  through a 'few' or 'scattered' cloud layer (i.e. <50% coverage).
  This is a quick hack rather than a proper fix, but it will at
  least make it possible to get above a scattered layer VFR.

2005-01-09 11:24  ehofman

* simgear/threads/: SGThread.cxx, SGThread.hxx: Add support for
  binding a thread to a specific CPU (IRIX only at this time).

2005-01-08 12:47  ehofman

* simgear/sound/sample_openal.cxx: Frederic Bouvier:

  It comes to me that the bulk of all problem reports, especially
  from Windows users, have it's cause in an obsolete sound driver.
  These messages should direct them to the right solution before
  complaining.

2005-01-03 20:05  curt

* NEWS, configure.ac: Updates for 0.3.8-pre2 release.

2004-12-23 14:32  ehofman

* simgear/props/: props.cxx, props.hxx, props_io.cxx: Melchior
  FRANZ:

  My recent fix for the load/save fgfs.sav feature was a bit too
  ambitious.  While aliases lead to abortion before, I tried to
  copy them properly, although this wasn't a requirement.
  Unfortunately, this seems to have worked for absolute aliases
  only, not for relative ones, and hence broke several panel
  instruments. The attached patch backs most of the previous patch
  out again, and goes a simpler route: just ignore aliases.

2004-12-19 11:19  ehofman

* simgear/props/: props.cxx, props.hxx, props_io.cxx: Melchior
  FRANZ:

  fgLoadFlight() loads a property file ("fgfs.sav") to a new
  property tree, and then copies that over to the main tree.
  copyProperties() didn't know how to handle type
  SGPropertyNode::ALIAS and hence threw an exception that made fgfs
  abort.

  The attached patch adds support for ALIAS to copyProperties():
  aliased nodes are created in the target tree if necessary, and
  then linked like in the source tree. It seemed useful to add an
  optional argument to props.[ch]xx/getType() that would indeed
  return the property type "ALIAS" for aliased nodes, and not the
  type of the node that it refers to. The patch also fixes a bug in
  writeNode() that caused extra lines after alias entries.

  If there's resistance to the change to getType() (David?) I can
  easily use isAlias(). This just makes copyProperties() a tad
  uglier, but I can live with it. It's useful for scanning a tree,
  though, if an alias node can be treated exactly like all other
  nodes, without automatic redirection.

2004-12-18 11:53  ehofman

* simgear/compiler.h: gcc 4.0 fix.

2004-12-16 14:15  ehofman

* simgear/sound/soundmgr_openal.cxx: Initialize volume to
  inaudiable at startup.

2004-12-15 17:45  curt

* configure.ac: More prerelease updates.

2004-12-15 17:34  curt

* Doxyfile, NEWS, configure.ac: Prerelease updates.

2004-12-13 21:31  ehofman

* configure.ac: Threads detection code cleanup and FreeBSD fixes.

2004-12-08 16:12  ehofman

* configure.ac: This was too  quick, now pthreads isn't detected on
  IRIX (and other platforms?) anymore. This needs some more
  thought.

2004-12-08 16:00  ehofman

* configure.ac: FreeBSD fix.

2004-12-05 10:36  ehofman

* simgear/serial/serial.cxx: AIX fix

2004-12-02 16:08  curt

* simgear/sound/soundmgr_openal.cxx: Oops, 2nd try ...

2004-12-02 16:00  curt

* simgear/: scene/sky/cloud.cxx, sound/soundmgr_openal.cxx: Martin
  Spott: Revised handling of missing isnan() on earlier versions of
  FreeBSD.

2004-12-01 18:37  curt

* simgear/compiler.h: Fix a typo for the Mac OSX platform.

2004-11-21 22:46  curt

* simgear/sound/soundmgr_openal.cxx: Fix a dumb bug.

2004-11-21 22:45  curt

* simgear/scene/sky/cloud.cxx: Fix a dumb bug for FreeBSD.

2004-11-21 18:05  ehofman

* simgear/sound/: soundmgr_openal.cxx, xmlsound.cxx: Melchior
  FRANZ:

  At last I've found the reason why fgfs crashed routinely for me.
  When I still used KDE's artsdsp (preloads lib with OSS
  replacement functions) I saw this crash only occasionally. After
  letting OpenAl communicate with artsd directly (by means of
  ~/.openalrc setting), I got the crash always when I left fgfs.

  This bug may also have crashed fgfs when running with sound
  daemons other than aRts.

2004-11-21 04:13  curt

* simgear/: scene/sky/cloud.cxx, sound/soundmgr_openal.cxx: I don't
  understand why FreeBSD doesn't see isnan() after including math.h
  but it doesn't.  Trying the apple approach to fixing isnan
  results in an infinite loop (making me wonder what happens on
  OSX?)  This is an alternative approach to checking isnan() on
  freebsd ...

2004-11-20 20:14  curt

* simgear/screen/extensions.cxx: FreeBSD fix.

2004-11-20 20:11  curt

* simgear/scene/sky/cloud.cxx: FreeBSD fix.

2004-11-19 22:47  curt

* simgear/: math/interpolater.cxx, math/interpolater.hxx,
  math/leastsqs.cxx, math/leastsqs.hxx, math/point3d.hxx,
  math/polar3d.cxx, math/polar3d.hxx, math/sg_random.c,
  math/sg_random.h, math/vector.cxx, math/vector.hxx,
  scene/material/matlib.cxx, scene/tgdb/obj.cxx,
  scene/tgdb/obj.hxx, timing/sg_time.cxx: Update a few more
  instances of my email address.

2004-11-19 22:44  curt

* Thanks, simgear/constants.h, simgear/version.h.in,
  simgear/bucket/newbucket.cxx, simgear/bucket/newbucket.hxx,
  simgear/ephemeris/ephemeris.cxx, simgear/ephemeris/ephemeris.hxx,
  simgear/ephemeris/stardata.cxx, simgear/ephemeris/stardata.hxx,
  simgear/io/iochannel.cxx, simgear/io/iochannel.hxx,
  simgear/io/lowlevel.cxx, simgear/io/lowlevel.hxx,
  simgear/io/sg_binobj.cxx, simgear/io/sg_binobj.hxx,
  simgear/io/sg_file.cxx, simgear/io/sg_file.hxx,
  simgear/io/sg_serial.cxx, simgear/io/sg_serial.hxx,
  simgear/io/sg_socket.cxx, simgear/io/sg_socket.hxx,
  simgear/io/sg_socket_udp.cxx, simgear/io/sg_socket_udp.hxx,
  simgear/magvar/magvar.cxx, simgear/magvar/magvar.hxx,
  simgear/math/localconsts.hxx, simgear/math/sg_types.hxx,
  simgear/misc/sg_path.cxx, simgear/misc/sg_path.hxx,
  simgear/misc/texcoord.cxx, simgear/misc/texcoord.hxx,
  simgear/scene/material/mat.cxx, simgear/scene/material/mat.hxx,
  simgear/scene/material/matlib.hxx,
  simgear/scene/material/matmodel.cxx,
  simgear/scene/material/matmodel.hxx,
  simgear/scene/model/location.cxx, simgear/scene/sky/cloud.cxx,
  simgear/scene/sky/cloud.hxx, simgear/scene/sky/dome.cxx,
  simgear/scene/sky/dome.hxx, simgear/scene/sky/sky.cxx,
  simgear/scene/sky/sky.hxx, simgear/scene/tgdb/apt_signs.cxx,
  simgear/scene/tgdb/apt_signs.hxx, simgear/scene/tgdb/leaf.cxx,
  simgear/scene/tgdb/leaf.hxx, simgear/scene/tgdb/pt_lights.cxx,
  simgear/scene/tgdb/pt_lights.hxx,
  simgear/scene/tgdb/userdata.cxx, simgear/scene/tgdb/userdata.hxx,
  simgear/scene/tgdb/vasi.hxx, simgear/serial/serial.cxx,
  simgear/serial/serial.hxx, simgear/sound/sample_openal.cxx,
  simgear/sound/sample_openal.hxx,
  simgear/sound/soundmgr_openal.cxx,
  simgear/sound/soundmgr_openal.hxx, simgear/sound/xmlsound.cxx,
  simgear/timing/sg_time.hxx, simgear/timing/timestamp.cxx,
  simgear/timing/timestamp.hxx: My old email address is no longer
  valid ... point to my web page.

2004-11-18 20:12  curt

* configure.ac: Ooops, fix an unintentional line wrap.

2004-11-18 20:10  curt

* simgear/: compiler.h, scene/material/matlib.cxx,
  scene/sky/dome.cxx, screen/GLBitmaps.cxx, screen/extensions.hxx,
  screen/screen-dump.cxx, screen/screen-dump.hxx,
  screen/texture.cxx, screen/texture.hxx, screen/tr.cxx,
  screen/tr.h: Abstract out location of gl.h, glut.h, and glu.h
  includes so that we can make the Mac platform happy since they
  put these in a different place compared to the rest of the world.

2004-11-17 20:37  andy

* simgear/structure/event_mgr.cxx: Make sure that timer delay
  values are positive-definite, otherwise user code that wants to
  use zero delay to mean "next frame" will get stuck in an infinite
  loop.

2004-10-24 11:29  ehofman

* simgear/debug/debug_types.h: Roy Vegard Ovesen:

  I've added two new debug log types for the instrumentation and
  systems. They used to use the autopilot debug log, because I
  couldn't figure out how to make new log types. Well, now I have
  figured it out.  ;-)

2004-10-17 19:06  ehofman

* simgear/scene/model/: model.cxx, model.hxx: Frederic Bouvier:

  This is a patch to make display list usage optional. They are on
  by default.  Use --prop:/sim/rendering/use-display-list=false to
  use immediate mode.  There is also a change in exception handling
  in main.cxx and bootstrap.cxx

2004-10-16 14:23  ehofman

* simgear/route/: waypoint.cxx, waypoint.hxx: Roy Vegard Ovesen:

  I'm working on a route manager in the GPS module. So I've added a
  name parameter to the waypoint class in Simgear. I use the
  existing ID parameter to store the ID, for example KLAX, and the
  name parameter to store the name, San Francisco Intl.

2004-10-14 15:35  ehofman

* configure.ac, simgear/scene/Makefile.am: Remove the refference to
  fgsg

2004-10-13 22:18  curt

* configure.ac: Fix a couple bugs in openal detection.	I should
  actually generate a new configure and test it, rather than
  testing the old configure script.

2004-10-13 21:52  curt

* configure.ac: Oops, missed a part of the previous change.

2004-10-13 21:51  curt

* README.OpenAL, configure.ac: Add a sanity check for the existance
  of OpenAL.  If not there, bail from the configure script with an
  appropriate/helpful message.

2004-10-12 16:35  curt

* Makefile.am, NEWS, configure.ac, simgear/scene/Makefile.am: Final
  0.3.7 changes.

2004-10-11 09:56  ehofman

* simgear/scene/model/model.hxx: Fix a typo.

2004-10-11 09:37  ehofman

* simgear/scene/model/: model.cxx, model.hxx: Frederic: Ignore
  display lists when using the blend animation.

2004-10-10 21:38  ehofman

* simgear/scene/model/model.cxx: Adding the panel was a step too
  far for Linux, causing a segfault.

2004-10-10 21:16  ehofman

* simgear/scene/tgdb/obj.cxx: Remove a phantom makeDList call
  (probably an old one from my code.

2004-10-10 21:05  ehofman

* simgear/scene/: model/model.cxx, tgdb/obj.cxx: Check for the plib
  version when using display lists, just to be sure.

2004-10-10 20:43  ehofman

* simgear/scene/model/model.cxx: Frederic: Include FGPanelNode in
  the display list generation process.

2004-10-10 19:49  ehofman

* simgear/scene/model/model.cxx: Frederic: Use display lists for 3d
  models also.

2004-10-06 11:57  ehofman

* simgear/screen/: jpgfactory.cxx, jpgfactory.hxx: Reverse the
  declaration order. jpgRenderFrame (formerly known as
  trRenderFrame) is now declared as a NULL function pointer and
  assignment of the proper function is now done in FlightGear
  (jpgRenderFrame=FGRenderer::update).

2004-09-30 11:43  ehofman

* simgear/screen/extensions.hxx: David Luff:

  The one-liner removes a lot of re-definition warnings on Cygwin.

2004-09-19 11:08  ehofman

* simgear/props/props.hxx: Small update for future use.

2004-09-15 17:28  curt

* simgear/threads/SGQueue.hxx: Expose the size() method for locked
  and blocking thread queues.

2004-09-15 17:28  curt

* simgear/sound/sample_openal.cxx: Fix another case where the
  direction vector is not initialized which can lead to openal
  "inrange" assertions, crashing FlightGear.

2004-09-10 22:44  curt

* simgear/sound/sample_openal.cxx: direction vector needs to be
  initialized, otherwise garbage data could cause openal to
  generate an assertion, aborting the top level app.

2004-09-10 20:16  curt

* Doxyfile, NEWS, configure.ac: Tweaks for 0.3.7-pre1

2004-09-10 17:57  curt

* simgear/: props/props_io.cxx, props/props_io.hxx,
  xml/easyxml.cxx, xml/easyxml.hxx: Add support for parsing an xml
  stream from an in memory buffer, rather than just from a
  specified file name.

2004-09-08 13:15  ehofman

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  xmlsound.cxx: Add support for audio orientation: direction and
  cone definition. This currently only works for internal view and
  tower view because those set the listener position correctly.

2004-08-17 10:31  ehofman

* simgear/math/: sg_geodesy.cxx, sg_geodesy.hxx: this simple patch
  will enable the direct use of Point3D::get_n() instead of making
  duplications to call sgCartToGeod

2004-08-17 10:28  ehofman

* simgear/scene/sky/sky.cxx: this simple patch will prevent to have
  fog in Clear cload layers.

2004-08-15 11:24  ehofman

* simgear/sound/soundmgr_openal.cxx: change the log level of the
  initialization of OpenAL. This prevent a console popup for no
  reason on Windows.

2004-07-29 23:36  curt

* NEWS, configure.ac: Tweaks for official 0.3.6 release.

2004-07-29 10:30  ehofman

* simgear/: io/sg_socket.cxx, xml/easyxml.cxx: Make gcc 2.95.*
  compile again.

2004-07-28 16:13  ehofman

* simgear/misc/sgstream.cxx: Use the SimGear default notation.

2004-07-28 15:59  ehofman

* simgear/: misc/sgstream.cxx, xml/easyxml.cxx: IRIX fixes (at
  least).

2004-07-28 15:16  ehofman

* simgear/io/sg_socket.cxx: IRIX fix (at least).

2004-07-27 23:18  curt

* NEWS, configure.ac: Tweaks for 0.3.6-pre3

2004-07-24 21:21  curt

* simgear/scene/model/animation.cxx: Fix a minor warning message.

2004-07-22 10:04  ehofman

* simgear/scene/model/animation.cxx: Correct a typo that produces
  segfault during cleanup on some systems.

2004-07-21 23:22  curt

* NEWS, configure.ac: Tweaks for 0.3.6-pre2

2004-07-21 12:52  ehofman

* simgear/props/props_io.cxx: Frederic Bouvier:

  I just discovered this : state() is not valid when _level==0,
  because it is doing: { return _state_stack[_state_stack.size() -
  1]; } and is returning a wrong index fetch ( 0 - 1 ) == -1

  Moreover, st is not used when _level==0 so the line can be moved
  to a safer place.

2004-07-19 16:59  ehofman

* configure.ac: Test for alutInit in libopenal.so and in libalut.so
  for Cygwin builds.

2004-07-17 16:01  ehofman

* configure.ac: alut is now part of libopenal.so

2004-07-16 23:00  ehofman

* simgear/sound/xmlsound.cxx: Weak excuse, but it's getting late.
  Do clipping right this time.

2004-07-16 22:36  ehofman

* simgear/sound/xmlsound.cxx: Clip to volume to 1.0 if necessary.

2004-07-15 00:01  curt

* Doxyfile, Makefile.am, NEWS, SimGear.dsp, configure.ac,
  simgear/io/sg_binobj.cxx, simgear/io/sg_binobj.hxx: Tweaks in
  preparation for the 0.3.6-pre1 release.

2004-07-05 18:39  ehofman

* simgear/misc/sg_path.cxx: Make sure that a directory name
  containing a '.' doesn't screw up the ::base() and ::extension()
  functions.

2004-07-05 15:40  ehofman

* simgear/scene/model/model.cxx: Remove the need to append a
  trailing / to the texture-path string.

2004-07-05 13:42  ehofman

* simgear/scene/model/model.cxx: Allow the use of a <texture-path/>
  tag to specify the directory containing the textures for this
  model.

2004-07-03 13:16  ehofman

* simgear/screen/extensions.hxx: Use #elif instead of #else if

2004-07-03 12:59  ehofman

* simgear/screen/extensions.cxx: Frederic Bouvier:

  I have 3 issues that are fixed by this set of patches.

  1. In extensions.cxx	 #else if !defined( WIN32 )  must be
  changed by   #elif !defined( WIN32 ) because the text after #else
    seems to be ignored

  2. banner is not available on windows, only cygwin

  3. ANSI escape sequences are not interpreted on the	 windows
  console. We just have garbage that is hard	to read.

2004-07-01 21:05  curt

* simgear/sound/soundmgr_openal.cxx: If we pass in a position or
  velocity of nan, openal will generate an assertion.  Under rare
  circumstances we could encounter a non-cooperative external fdm
  that could trigger this condition.  This patch catches the
  problem and returns rather than letting FG get killed.

2004-06-27 10:09  ehofman

* simgear/screen/extensions.cxx: Some small updates.

2004-06-27 09:49  ehofman

* simgear/screen/: extensions.cxx, extensions.hxx: Work around a
  broken dlopen/dlclose Linux implementation.

2004-06-25 12:59  ehofman

* simgear/scene/sky/cloud.cxx: Frederic Bouvier:

  this patch correct the cloud repositionning that made them going
  against the wind. Now the clouds and the windsock show the same
  direction.

2004-06-15 14:47  david

* simgear/sound/.cvsignore: Ignore generated files.

2004-06-13 13:59  ehofman

* simgear/sound/xmlsound.cxx: Be a bit anoying (and a tad more
  descriptive) about OpenAL volume errors for some time.

2004-06-12 23:03  ehofman

* simgear/scene/tgdb/obj.cxx: Frederic Bouvier:

  Do state sorting by material before adding primitives in the tile
  branch. I thought I could see a bit of improvement in framerate
  but it is not the case on my setup.  I don't see a degradation
  though

2004-06-07 20:49  ehofman

* simgear/scene/sky/cloud.cxx: Frederic Bouvier:

  I am still experimenting with the code. Here is so far the best I
  could achieve. The dark aspect of clouds at dusk or dawn is far
  better than the problems of transparency of the previous version.

2004-06-07 14:43  ehofman

* simgear/misc/sg_path.cxx: MingW fix.

2004-06-07 11:50  ehofman

* simgear/misc/: sg_path.cxx, sg_path.hxx: Add an 'add(str)'
  function that adds a search path separator and appends the str.

2004-06-04 18:59  ehofman

* simgear/sound/soundmgr_openal.cxx: Tweak the doppler effect.

2004-05-28 10:42  ehofman

* simgear/props/props.hxx: Revert the previous patch. There was
  already a class availble that allows for that. It's just that the
  description doesn't explain too much.

2004-05-27 20:00  ehofman

* simgear/scene/sky/cloud.cxx: Tweak the bump-mapped 2d cloud color
  a bit.

2004-05-27 15:03  ehofman

* simgear/props/props.hxx: Add the possibillity to parse a user
  data pointer to getter and satter functions. This adds a
  convenient way to get the 'this' pointer to the static functions.

2004-05-25 09:58  ehofman

* simgear/scene/sky/cloud.cxx: Use a different coloring scheme.

2004-05-21 18:27  ehofman

* simgear/scene/sky/cloud.cxx: MacOSX fix that never got applied
  before.

2004-05-21 17:07  ehofman

* simgear/scene/sky/cloud.cxx: mingw32 fix

2004-05-21 16:50  ehofman

* simgear/: scene/sky/cloud.cxx, screen/extensions.hxx: Updates
  from Frederic to use 2 texture units and a color blend function
  rather than 3 texture units.

2004-05-20 16:18  ehofman

* simgear/scene/model/animation.cxx: Frederic Bouvier:

  Melchior spotted a problem where we can crash an airplane into
  the beacon's beam. The patch below enable to mask out a branch
  from HOT traversal, whatever the animation.

  The beacon.xml file is also included. It has a <enable-hot
  type="bool">false</enable-hot> in a halo branch

2004-05-20 16:02  ehofman

* simgear/scene/sky/cloud.cxx: Make sure there will be no previous
  declaration errors.

2004-05-20 15:24  ehofman

* simgear/: screen/extensions.hxx, screen/texture.cxx,
  screen/texture.hxx, scene/sky/cloud.cxx, scene/sky/cloud.hxx:
  Patch from Frederic. Adds support for bump-mapped (multi
  textured) 2d clouds, includeing support code.

2004-05-15 14:45  ehofman

* simgear/scene/model/model.cxx: Fred: include more check against
  null pointers and a raise in log level for missing objects.

2004-05-14 21:46  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Solve the endless loop problem for the DC-3 and prevent a
  potential segmentation fault.

2004-05-14 17:47  curt

* simgear/sound/soundmgr_openal.hxx: Add a function to globally set
  volume (aka AL_GAIN)

2004-05-12 00:39  curt

* simgear/scene/model/custtrans.cxx: Oops, fix a typo.

2004-05-12 00:21  curt

* simgear/scene/: material/matmodel.cxx, model/Makefile.am,
  model/animation.cxx, model/animation.hxx, model/custtrans.cxx,
  model/custtrans.hxx, model/flash.cxx, model/flash.hxx,
  model/model.cxx: Frederic Bouvier:

  I was not very happy with the size of the halo, so I created a
  new animation to control it. Now we can control the scale value
  with the distance from the viewer to the object. The towers are
  now beginning to look good. They might need some tuning though.
  If you want to play, locate in radio-*.xml this code :

    <animation>
    <type>dist-scale</type>
    <object-name>RedLight.2</object-name>
    <interpolation>
      <entry><ind>0</ind><dep>0.1</dep></entry>
      <entry><ind>500</ind><dep>0.3</dep></entry>
      <entry><ind>16000</ind><dep>3</dep></entry>
    </interpolation>
    </animation>

  You get the idea ? ind is the distance, dep is the resulting
  scale value.	The medium tower appears brighter than the tall
  one, because the lights are closer to each other. Maybe they need
  a smaller scale factor at distance. Feel free to modify these
  values if you find a better setup.

  About the code : I renamed flash to custtrans because the ssg
  branch is now less specialized. It needs a callback to compute
  the so called 'custom transformation'. It can be used for the
  SGFlashAnimation and the new SGDistScaleAnimation. So please cvs
  remove flash.[ch]xx and add custtrans.[ch]xx. I also undo some of
  the code I send you yesterday that was totally useless. It is
  replaced by something simpler and that works.

  There is also a patch to matmodel.cxx. This is not related and
  was something I forgot. Its purpose is to set the alpha test on
  material billboard models that are likely to be trees to lessen a
  transparency weird effect with clouds.

2004-05-10 23:22  curt

* simgear/sound/: Makefile.am, sample_openal.cxx,
  sample_openal.hxx: I had overlooked a few memory
  allocation/deallocation issues for audio buffers.  Hopefully this
  helps clean those up.

2004-05-10 22:27  curt

* simgear/scene/model/animation.cxx: Frederic Bouvier:

  I modified the included animation.cxx to have a randomly
  displaced time origin, to break the unison. And the flashing
  period is also random as you noticed. I also put all the flashing
  light of the pole in the same animation so they flash in the same
  rhythm.

2004-05-10 16:59  curt

* simgear/scene/model/: Makefile.am, animation.cxx, animation.hxx,
  flash.hxx, modellib.cxx, personality.cxx, personality.hxx:
  Frederic Bouvier:

  Fix a memory leak, and brownian animation, if not motion.

  I have 2 new files : personality.[ch]xx . They store the
  personality data that would be deleted when the object is
  destroyed, instead of staying in the animation maps. I also
  manage the current animation step better and the towers are not
  flashing randomly now.  Makefile.am is updated.

2004-05-10 16:35  curt

* simgear/scene/model/: animation.cxx, animation.hxx, flash.cxx,
  flash.hxx, modellib.cxx: Frederic Bouvier:

  modellib.cxx :   Add a branch between the model and its
  transformation to add  a unique identifier for the model. I
  called it "personality  branch" and it maintains a
  "current_object" in SGAnimation.   Animations specifically
  written to support it ( currently only   the timed animation )
  can add a degree of variety among the   occurrences of the same
  model that would look otherwise cloned.

  flash.[ch]xx :   Better compute the view vector. The flash is now
  syncronized with  its axis even at the edge of the screen.

  animation.[ch]xx :   Lots of changes :  - add a condition to
  'rotate', 'spin', 'translate' and 'range'.	When a condition is
  specified *and* it evaluates to false, the	animation becomes a
  no-op. Possible usage : no rotation during	daylight, LOD range
  variable along the day, ...

    - use different durations for each branch in the timed
  animation.
    Enable the use of multiple <branch-duration-sec>, one for each
    <object-name> specified. Usage : strobes with flash light.

    - allow randomization of the <branch-duration-sec>, by using
    <random><min>_min_value_</min><max>_max_value_</max></random>.
    The value computed once is between _min_value_ and _max_value_.

    - implement model personality in timed animation. If
    <use-personality type="bool">true</use-personality> is
  specified,
    a different set of duration is created for every model in the
    scenegraph using this animation. Best if used with
  randomization.
    When using strobes where the population of the same object is
    dense, it avoids the "cheasy" clone effect.

2004-05-08 14:58  ehofman

* simgear/math/: fastmath.cxx, fastmath.hxx: Add fast functions for
  exp2, pow, log2, root, sin/cos/tan, asin/acos/atan along with
  abs, neg and sgn.

2004-05-07 18:42  ehofman

* simgear/scene/model/: Makefile.am, animation.cxx, animation.hxx,
  flash.cxx, flash.hxx, model.cxx: Frederic Bouvier:

  this patch introduce a new kind of animation and ssg branch.	I
  called them flash animation, because they help me to enhance the
  look of the rotating beacon and possible future lighthouse. It
  computes the cosine of the angle between an arbitrary axis,
  transformed by the current modelview matrix, and the view
  direction. No trig involved, just a dot/scalar product.

  The computed value can be modified by three parameters, power,
  factor and offset, according to the formulae :

    value = factor * pow( cosine, power ) + offset.

  It is clamped between a minimum and a maximum.  This value is
  then used as the scale factor of a matrix transformation applied
  to the children of the SGFlash branch.

  The xml syntax, with default values, is :

  <animation>  <type>flash</type>
  <object-name>HaloObject</object-name>  <center>   <x-m>0</x-m>
  <y-m>0</y-m>	 <z-m>0</z-m>  </center>  <axis>   <x>0</x>
  <y>0</y>   <z>1</z>  </axis>	<power>1</power>
  <factor>1</factor>  <offset>0</offset>  <min>0</min>
  <max>1</max>	<two-sides>false</two-sides> </animation>

2004-05-03 20:43  andy

* simgear/structure/event_mgr.cxx: Robustify the SGTimerQueue
  destructor.  There have been reports of crashes on deletion.

2004-05-03 20:39  andy

* simgear/structure/event_mgr.hxx: Add a destructor for SGEventMgr.
  We don't own the pointer, so we can't free it.  Just zero it
  out.

2004-04-30 02:44  andy

* configure.ac, simgear/compiler.h, simgear/scene/sky/cloud.cxx,
  simgear/sound/openal_test1.cxx, simgear/sound/openal_test2.cxx,
  simgear/sound/sample_openal.hxx: Changes to get SimGear to
  configure and compile out-of-the-box on a MinGW target:

  Link against alut.dll in addition to openal32.dll.

  Remove some preprocessor defines from compiler.h that were
  confusing the mingw and/or libstdc++ headers (I put the _isnan
  one back in the only file it was used).

  Hack a broken sleep() call into the OpenAL sample programs so
  that they will compile (but not work) in a non-POSIX environment.

  Change the header file ordering in sample_openal.hxx to get
  around some really weird interactions between MinGW's windows.h
  and the gcc iostream header.

2004-04-29 23:14  curt

* simgear/sound/Makefile.am: Hopefully fix a chicken/egg linking
  problem for people who've never built or installed simgear
  before.

2004-04-28 22:37  curt

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  xmlsound.cxx: Add support for specifying a positional offset
  relative to the listener.  This allows us to "place" cockpit
  sounds.  For example, we can make left engine sound come out of
  the left speaker, right engine out the right speaker, etc.

2004-04-28 21:19  curt

* configure.ac: Add default openal libs for cygwin.

2004-04-28 05:59  curt

* simgear/sound/xmlsound.cxx: Lower verbosity level.

2004-04-28 05:57  curt

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  xmlsound.cxx: Expose the ability to specify how the sound volume
  fades relative to distance from the listener.  This let's us
  configure "interior" cockpit sounds versus "exterior" engine type
  sounds.

2004-04-27 23:11  curt

* simgear/sound/sample_openal.cxx: Tweak default source audio
  parameters.

2004-04-27 22:45  curt

* simgear/sound/: sample_openal.cxx, sample_openal.hxx,
  soundmgr_openal.cxx, soundmgr_openal.hxx: Expose some of the
  positional components of the OpenAL API.

2004-04-27 10:59  ehofman

* simgear/sound/: openal_test1.cxx, openal_test2.cxx: Make sure the
  prototype definition of sleep() is found (at least for IRIX).

2004-04-27 00:07  curt

* simgear/sound/sample_openal.cxx: Oops, one addtional tweak.

2004-04-27 00:02  curt

* simgear/sound/: openal_test2.cxx, sample_openal.cxx,
  sample_openal.hxx, xmlsound.cxx: Update the SoundSample api so we
  can request that a copy of the sample be kept in memory and
  accessible.

2004-04-26 18:17  curt

* simgear/bucket/newbucket.hxx: Give these two methods slightly
  less misleading names.

2004-04-26 17:55  curt

* simgear/bucket/newbucket.hxx: David Luff:

  Fix comments for two functions.

2004-04-26 16:55  curt

* simgear/sound/openal_test1.cxx: Missed one fix for Mac OS.

2004-04-25 04:30  curt

* simgear/sound/sample_openal.cxx: Lower the verbosity in a couple
  other spots.

2004-04-25 04:02  curt

* simgear/sound/sample_openal.cxx: Lower verbosity.

2004-04-25 03:48  curt

* simgear/sound/: Makefile.am, openal_test1.cxx, sample_openal.cxx,
  sample_openal.hxx, soundmgr_openal.cxx, soundmgr_openal.hxx: Add
  support for the MacOS variations of OpenAL.

2004-04-25 03:41  curt

* configure.ac: Add correct openal libs for MacOS.

2004-04-24 21:47  curt

* simgear/sound/sample_openal.hxx: Clamp pitch values rather than
  just dumping an error message.

2004-04-24 21:02  curt

* configure.ac, simgear/sound/Makefile.am, simgear/sound/jet.wav,
  simgear/sound/openal_test1.cxx, simgear/sound/openal_test2.cxx,
  simgear/sound/sample_openal.cxx, simgear/sound/sample_openal.hxx,
  simgear/sound/sound.cxx, simgear/sound/sound.hxx,
  simgear/sound/soundmgr.cxx, simgear/sound/soundmgr.hxx,
  simgear/sound/soundmgr_openal.cxx,
  simgear/sound/soundmgr_openal.hxx, simgear/sound/xmlsound.cxx,
  simgear/sound/xmlsound.hxx: Rewrite the entire audio support
  library on top of OpenAL rather than plib's sound manager.  The
  interface was simplified and cleaned up a bit, and I haven't back
  ported these changes to the plib sound wrappers ... we could I
  suppose if someone really had a problem, but I haven't seen
  anything so far that would indicate the extra effort is worth it.

2004-04-22 14:39  curt

* simgear/threads/SGQueue.hxx: Bernie Bright:

  gcc 3.4 has changed the rules for unqualified template name
  lookup.  This affects SGQueue.hxx.  The changes I've made are
  backwards compatible with earlier gcc versions.  Everything else
  compiles pretty much okay except for a few warnings.	The
  resultant executable seems a bit faster too.

2004-04-04 17:35  ehofman

* simgear/scene/sky/cloud.cxx: Frederic: The state selector was not
  referenced and got deleted as soon as the sky was rebuilt a
  second time with the metar code.

2004-04-04 16:24  david

* .cvsignore: Added more generated files.

2004-04-04 15:46  ehofman

* simgear/screen/screen-dump.hxx: Make sure GLuint is known.

2004-04-04 15:41  ehofman

* simgear/scene/sky/: cloud.cxx, cloud.hxx, sky.cxx: Frederic
  Bouvier:

  This is a new patch that allow to define a different texture for
  top and bottom of clouds. It uses the overcast_top.rgb you made
  for me last time.

  What the patch do is to install a ssgStateSelector instead of a
  ssgSimpleState for each layer.  The SGCloudLayer::draw method is
  modified to accept a boolean that will select the proper state:
  0/false for bottom, 1/true for top.

  Then, in SGSky::drawUpperClouds and SGSky::drawLowerClouds,
  SGCloudLayer::draw is called with false and true because we see
  the bottom of upper clouds and the top of lower clouds.

  Only overcast has 2 textures, the other types share the same
  state for top and bottom, but that could be modified in
  SGCloudLayer::rebuild.

2004-04-02 21:48  ehofman

* simgear/scene/model/model.cxx: Plib is willing callbacks to
  return 0, 1 or 2 and not simply a boolean

2004-04-02 21:44  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Fix an NVidia problem by
  moving the hack to another location.

2004-04-02 16:39  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Frederic Bouvier:

    Move the rendering stage for upper clouds from preDraw
    to drawUpperClouds. Rename postDraw to drawLowerClouds.

2004-04-02 16:39  ehofman

* simgear/scene/tgdb/obj.cxx: Frederic Bouvier:

    put all leaf is a seperated branch so that it is
    possible to use a pretrav callback to cull out
    terrain without culling out light or dynamic
    objects. It appears that plib is not calling the
    pretrav callback for leaves.

2004-04-02 16:38  ehofman

* simgear/scene/model/: model.cxx, model.hxx:
  Frederic Bouvier:

    add an optional parameter that would be called to
    build the aircraft panel, so that flightgear's
    model_panel no longer duplicate code.

    add a pretrav callback to models so that we can
    filter out models when calling ssgCullAndDraw on
    the global scene.
    sgSetModelFilter( true ) means that we want to draw
    the models. Use false to cull them out.

2004-04-01 15:47  curt

* simgear/: math/fastmath.cxx, scene/material/mat.cxx,
  scene/material/mat.hxx, scene/material/matlib.cxx,
  scene/model/animation.cxx, scene/sky/oursun.cxx,
  scene/sky/oursun.hxx: Clean up several stray warnings that have
  accumulated.

2004-03-26 22:07  curt

* NEWS, configure.ac: Final 0.3.5 tweaks.

2004-03-24 01:19  curt

* NEWS, configure.ac: 0.3.5.pre2 tweaks.

2004-03-23 20:04  david

* .cvsignore: Ignore generated files.

2004-03-23 01:42  andy

* simgear/nasal/code.c: Fix GC interaction.  The "me" reference to
  a method call was being popped off the stack in a situation where
  it could be garbage collected.  I believe this is the source of
  the spurious "non-object have no members" bug that has been
  reported.

2004-03-22 22:31  ehofman

* simgear/scene/sky/cloud.cxx: Let the cloud layers follow the
  earth's surface. If you want the clouds to touch the horizon,
  just increase the spaen.

2004-03-22 21:03  ehofman

* simgear/screen/colors.hxx: These test should not be needed when
  properly using OpnGL colors.

2004-03-22 20:12  curt

* NEWS, SimGear.dsp, configure.ac: Tweaks for 0.3.5.pre1 release.

2004-03-22 20:12  curt

* simgear/screen/colors.hxx: Better color component sanity
  checking.

2004-03-21 22:54  ehofman

* simgear/misc/sg_path.cxx: Frederic Bouvier:

  If the input string of sgSplitPath is empty, it returned a list
  with one empty string, not an empty list.

2004-03-20 23:41  ehofman

* simgear/scene/model/location.cxx: Frederic Bouvier:

  The message 'Alert: catching up on tile delete queue' comes from
  the fact that 48 tiles are scheduled and added to the cache at
  startup before the plane location is initialized. My proposed
  patch is to initialize SGLocation with an invalid position and
  detect this fact before scheduling tiles. I prefer to do that
  rather than testing for lon and lat being 0,0 because it is a
  valid position and someone could want to fly near Accra.

2004-03-20 23:38  ehofman

* simgear/debug/logstream.hxx: Frederic Bouvier:

  This patch is for windows only. It hides the console window until
  there is a message to print. It only support SG_LOG, that I think
  is the right way to display something in FG.

2004-03-18 10:25  ehofman

* simgear/screen/extensions.hxx: Another Cygwin fix. This seems to
  take care of things proeprly.

2004-03-17 16:20  ehofman

* simgear/scene/model/animation.cxx: Disable deselecting the branch
  if it becomes translucent because it crashes the UFO.

2004-03-17 11:45  ehofman

* configure.ac, simgear/Makefile.am: Remove an unused library since
  FlightGear dropped support for WeatherCM. Users are highly
  encouraged to use environment/metar instead.

2004-03-17 11:31  ehofman

* simgear/screen/: GLBitmaps.cxx, extensions.hxx, screen-dump.cxx:
  Attempt to fix the Cygwin build problem for once and for all

2004-03-17 05:22  curt

* Doxyfile, NEWS, SimGear.dsp, configure.ac,
  simgear/compatibility/Makefile.am, simgear/nasal/Makefile.am:
  Various changes in preparation for the 0.3.5 release.

2004-03-12 19:55  ehofman

* simgear/scene/sky/oursun.cxx: Limmit the change in sun color due
  to visibility to a saner range.

2004-03-12 18:38  ehofman

* simgear/scene/model/animation.cxx: Use a more clever way to
  deselect a fully translucent leaf.

2004-03-12 11:09  ehofman

* simgear/scene/model/animation.cxx: Unselect the branch if the
  object has become fully translucent.

2004-03-12 09:59  ehofman

* simgear/misc/zfstream.hxx: Update for non-conformal (older)
  compilers

2004-03-08 09:59  ehofman

* simgear/scene/sky/cloud.cxx: MacOS X refinement

2004-03-07 19:47  ehofman

* simgear/scene/material/mat.cxx: Silently ignore texture files
  that are not present.

2004-03-07 10:36  ehofman

* simgear/: environment/metar.cxx, misc/zfstream.hxx: MSVC .NET
  2003 fix

2004-03-07 10:28  ehofman

* simgear/scene/sky/cloud.cxx: MacOS X 10.3 fix

2004-03-03 22:37  curt

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Work around a
  limitation of gc_calc_course_dist().	When the start and end
  points are too close together, this routine can return a course
  of "nan" but the distance is valid.  Someday someone who
  understands the math should rewrite the gc_calc_course_dist()
  routine so it is more robust, but for now it's easiest to simply
  check for a nan result and code around the limitation.

2004-03-03 22:35  curt

* simgear/compiler.h: #define isnan _isnan for MSVC and Mingwin.

2004-03-03 21:06  ehofman

* simgear/props/props.cxx: Remove a typo

2004-03-03 21:05  ehofman

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Put Curt's cloud
  movement fix back in

2004-03-03 20:59  ehofman

* simgear/props/props.cxx: MSVC .NET 2003 fix.

2004-03-03 20:54  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx: Add const back in the
  function

2004-03-03 20:48  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx: Some small fixes

2004-03-02 16:18  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx: A real MSVC fix this
  time

2004-03-02 15:58  ehofman

* simgear/scene/material/mat.cxx: Revert the last change, MSVC
  still doesn't like it.

2004-03-02 15:49  ehofman

* simgear/scene/material/mat.cxx: MSVC fixes

2004-03-02 14:28  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx: Use a round-robin
  scheme rather than returning a random textured state.

2004-03-02 11:51  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx: Make it possible to
  define multiple texture for a material (multiple <texture>
  entries in the materials.xml file). The code can return a random
  texture, or a pre defined texture upon request (default is
  random)

2004-02-28 19:51  curt

* simgear/threads/SGQueue.hxx: Add front() methods SGQueue,
  SGLockedQueue, and SGBlockingQueue so that the can be used more
  interchangably with a regular STL queue.

2004-02-27 04:30  david

* simgear/scene/model/model.cxx: Load submodels before animations,
  so that we can animate submodels.

  Allow submodels to be named when they are loaded.

2004-02-26 10:46  ehofman

* simgear/environment/: metar.cxx, metar.hxx: Melchior FRANZ:

  Add proxy support to the metar class. Authorization is untested,
  but everything else works. Martin will have to tell us ...

2004-02-23 21:07  ehofman

* simgear/environment/: metar.cxx, metar.hxx: Melchior FRANZ: Here
  are some updates for the metar class:

  - support for empty visibility group - support for 4-alnum IACO
  ids (rather than only 4-alpha) - better diagnosis in exception
  messages - check for 404 http response - replace last constant by
  constants.h definition

2004-02-23 02:34  curt

* simgear/environment/metar.hxx: Comment out an improperly written
  constructor.

2004-02-20 17:07  ehofman

* simgear/threads/SGThread.cxx: An ugly hack to get MipsPro 7.4.1
  working on IRIX 6 .5.20 (Yuck)

2004-02-20 16:10  andy

* simgear/nasal/misc.c: Fix from Richard Harke for 64 bit systems.
  The reftag was left uninitialized by naNum().  If it happened to
  be constructing it on the stack in a location previously occupied
  by a real reference, it would generate a corrupt naRef.

2004-02-18 15:33  ehofman

* simgear/scene/sky/: dome.cxx, oursun.cxx: Changes to tke sky dome
  coloring

2004-02-17 15:40  ehofman

* simgear/scene/tgdb/leaf.cxx: Remove an extra sgSetVec call

2004-02-07 22:36  david

* simgear/io/: sg_serial.cxx, sg_socket.cxx, sg_socket_udp.cxx: Be
  a little quieter at the default debug level.

2004-02-02 11:12  ehofman

* configure.ac, simgear/Makefile.am,
  simgear/environment/.cvsignore, simgear/environment/Makefile.am,
  simgear/environment/metar.cxx, simgear/environment/metar.hxx:
  Move the new metar class from FlightGear to SimGear

2004-02-01 18:47  andy

* simgear/nasal/lib.c: Yank the MSVC special handling.	It turns
  out it was because "strlen" has special voodoo in the parser.
  That's much more cleanly handled by renaming the function than by
  #ifdef'ing.

2004-01-31 20:50  curt

* simgear/route/: waypoint.cxx, waypoint.hxx: Fix a slight
  ambiguity in variable names.

2004-01-29 19:25  ehofman

* simgear/scene/sky/: moon.cxx, sky.cxx: Activate the driver fog
  workaround again. It doesn't seem to be solved yet.

2004-01-27 16:55  curt

* simgear/serial/serial.cxx: Frederic BOUVIER:

  Win32 serial port communication fixes.

2004-01-27 10:41  ehofman

* configure.ac: Make sure all libraries are used to test for
  certain functions

2004-01-26 20:59  ehofman

* simgear/constants.h: Fix a mistake

2004-01-24 13:08  ehofman

* simgear/scene/sky/: moon.cxx, oursun.cxx, sky.cxx, stars.cxx:
  Clean up the Pre-, and PostDraw functions a bit. Especially the
  glPushAttrib has had some attention. The NVidia hack is commented
  out for now.

2004-01-19 14:37  ehofman

* simgear/scene/sky/moon.cxx: Fix an initialization problem

2004-01-16 18:37  curt

* simgear/scene/tgdb/vasi.hxx: Oops, I originally had ramped the
  vasi/papi color transition the wrong way.  So as you passed
  through the target glide slope from low to high it would be
  colored: red -> white -> small range of transition to red ->
  white.  Now it goes the right way so you get: red -> smooth
  transition to -> white.  You can tell you are getting high if you
  see the bottom vasi start to turn pink ... etc. etc. hopefully
  just like in real life.

2004-01-15 15:23  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Try to prevent a devide by
  zero situation

2004-01-15 15:22  ehofman

* simgear/screen/colors.hxx: Don't do any calculations when thay
  are not needed

2004-01-15 15:21  ehofman

* simgear/compiler.h: Make sure MispPro compilers < 7.4 still work

2004-01-14 19:02  curt

* simgear/scene/sky/moon.cxx: The emissive values just don't seem
  like the right thing to do.  You suddenly see the dark side of
  the moon quite clearly, which usually isn't the case.

  The rest of the moon still seems a bit oversaturated right now
  ...

2004-01-09 17:49  curt

* simgear/scene/tgdb/pt_lights.cxx: Make the vasi lights slightly
  larger/brighter.

2004-01-09 11:19  ehofman

* simgear/scene/sky/: moon.cxx, moon.hxx: Use the same coloring
  scheme for the moon and the sun, add a bit of light reflection to
  the moon

2004-01-08 11:38  ehofman

* simgear/scene/model/animation.hxx: Updates to the alpha-test
  animation class

2004-01-08 11:25  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Updates to the alpha-test animation class

2004-01-07 10:07  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Frederic Bouvier:

  The attached patch adds a new animation class, called
  SGAlphaTestAnimation, to enable alpha test in 3D models.  This
  will remove many artefact caused by painting order of translucent
  models, and I need it for an upcoming model. In addition, I added
  a min-factor and a max-factor to the range animation to modulate
  min-m, min-property, max-m or max-property.

2003-12-30 08:04  curt

* simgear/scene/tgdb/: pt_lights.cxx, vasi.hxx: Smarter vasi
  coloring.

2003-12-30 06:53  curt

* simgear/scene/tgdb/: Makefile.am, obj.cxx, pt_lights.cxx,
  pt_lights.hxx, vasi.hxx: Track some additional data required for
  computing vasi/papi colors.

2003-12-30 06:53  curt

* simgear/scene/material/matlib.cxx: Go with an all white texture
  for vasi/papi lights.  We will vary the background color
  externally to change from white to red.

2003-12-27 14:41  ehofman

* simgear/constants.h: Add a number of Metar related constants

2003-12-27 10:31  ehofman

* simgear/io/Makefile.am: Resolve some dependency problems

2003-12-26 14:55  ehofman

* configure.ac, simgear/compatibility/Makefile.am,
  simgear/compatibility/README, simgear/compatibility/cassert,
  simgear/compatibility/cctype, simgear/compatibility/cerrno,
  simgear/compatibility/cfloat, simgear/compatibility/climits,
  simgear/compatibility/clocale, simgear/compatibility/cmath,
  simgear/compatibility/csetjmp, simgear/compatibility/csignal,
  simgear/compatibility/cstdarg, simgear/compatibility/cstddef,
  simgear/compatibility/cstdio, simgear/compatibility/cstdlib,
  simgear/compatibility/cstring, simgear/compatibility/ctime,
  simgear/compatibility/cwchar, simgear/compatibility/cwctype,
  simgear/compatibility/fstream, simgear/compatibility/iomanip,
  simgear/compatibility/iostream,
  simgear/compatibility/irix_string, simgear/compatibility/istream,
  simgear/compatibility/iterator, simgear/compatibility/new,
  simgear/compatibility/sstream, simgear/compatibility/streambuf,
  simgear/compatibility/strstream,
  simgear/compatibility/MIPSpro721/.cvsignore,
  simgear/compatibility/MIPSpro721/Makefile.am,
  simgear/compatibility/MIPSpro721/fstream,
  simgear/compatibility/MIPSpro721/iomanip,
  simgear/compatibility/MIPSpro721/iostream,
  simgear/compatibility/MIPSpro721/irix_string,
  simgear/compatibility/MIPSpro721/istream,
  simgear/compatibility/MIPSpro721/iterator,
  simgear/compatibility/MIPSpro721/new,
  simgear/compatibility/MIPSpro721/sstream,
  simgear/compatibility/MIPSpro721/streambuf,
  simgear/compatibility/MIPSpro721/strstream,
  simgear/compatibility/MIPSpro740/.cvsignore,
  simgear/compatibility/MIPSpro740/Makefile.am,
  simgear/compatibility/MIPSpro740/README,
  simgear/compatibility/MIPSpro740/cassert,
  simgear/compatibility/MIPSpro740/cctype,
  simgear/compatibility/MIPSpro740/cerrno,
  simgear/compatibility/MIPSpro740/cfloat,
  simgear/compatibility/MIPSpro740/climits,
  simgear/compatibility/MIPSpro740/clocale,
  simgear/compatibility/MIPSpro740/cmath,
  simgear/compatibility/MIPSpro740/csetjmp,
  simgear/compatibility/MIPSpro740/csignal,
  simgear/compatibility/MIPSpro740/cstdarg,
  simgear/compatibility/MIPSpro740/cstddef,
  simgear/compatibility/MIPSpro740/cstdio,
  simgear/compatibility/MIPSpro740/cstdlib,
  simgear/compatibility/MIPSpro740/cstring,
  simgear/compatibility/MIPSpro740/ctime,
  simgear/compatibility/MIPSpro740/cwchar,
  simgear/compatibility/MIPSpro740/cwctype,
  simgear/props/props.cxx: Updates to support sgi's MIPSpro
  compiler version 7.4 and newer. This also fixes a number of C++
  issues where FlightGear didn't follow the standard.

2003-12-22 20:27  andy

* simgear/nasal/: code.c, code.h, codegen.c, parse.c: Parse/GC
  interaction fixed.  Remove the OP_NEWARGS "optimization" (it
  wasn't).

2003-12-19 18:44  ehofman

* simgear/structure/exception.cxx:
  Melchior FRANZ: There are constructors for sg_io_exception that
  don't initialize the location. In this case getFormattedMessage
  does still output an extra line with " at" followed by ...
  nothing. Looks silly.

2003-12-19 03:40  andy

* simgear/: scene/model/location.cxx, math/sg_geodesy.cxx,
  math/sg_geodesy.hxx, scene/tgdb/obj.cxx: Rewrite sg_geodesy.	The
  new version is more accurate to the WGS84 standard and includes a
  sgCartToGeod() function which is 100% symmetric (down to the
  precision of a double) with sgGeodToCart().

2003-12-14 15:45  ehofman

* simgear/io/: sg_socket.cxx, sg_socket.hxx: Melchior FRANZ:

  There's another problem with sg_socket.cxx: the timeout value is
  hardcoded as 0. This is appropriate for servers as they
  continuously poll the socket and never want to wait for clients.
  But clients *do* want to wait a few seconds for a server
  response.

2003-12-14 11:07  ehofman

* simgear/io/sg_socket.cxx: Make sure both client and server
  situations are hendled properly

2003-12-11 16:26  ehofman

* simgear/compatibility/: fstream, iostream: Revert to the previous
  version

2003-12-11 14:17  ehofman

* simgear/: compiler.h, compatibility/fstream: fgrun fixes

2003-12-08 17:55  andy

* simgear/nasal/lex.c: Extra i++ led to a "double skip" when
  parsing a \r\n pair.	So a DOS file would look like it was all on
  one line, and a comment would skip to the end of the file.

2003-12-07 20:53  andy

* simgear/structure/: event_mgr.cxx, event_mgr.hxx: Bug fixes.	The
  priority queue wasn't handling boundary conditions at the edge of
  the table properly.  The new code is half the size -- it *has* to
  be correct, right?

2003-12-05 02:49  andy

* simgear/misc/: Makefile.am, interpolator.cxx, interpolator.hxx:
  Property interpolator subsystem.  A utility, primarily for Nasal
  scripts

2003-12-05 02:49  andy

* simgear/nasal/: lib.c, nasal.h, vector.c: Added setsize() and
  subvec() library functions

2003-12-01 19:57  ehofman

* simgear/nasal/: Makefile.am, nasal.h: A first attempt on
  detecting endianness and number of bits

2003-12-01 19:56  ehofman

* simgear/: compiler.h, compatibility/IRIX,
  compatibility/Makefile.am, compatibility/irix_string,
  misc/tabbed_values.hxx: Rename IRIX to irix_string in
  simgear/compatibility and assign
  "simgear/compatibility/irix_string" to STL_STRING for IRIX

2003-12-01 15:33  ehofman

* simgear/nasal/: code.c, data.h, gc.c, hash.c, lex.c, lib.c,
  misc.c, nasal.h: Add Nasal Vs. 1.5

2003-11-27 14:24  curt

* simgear/io/sg_socket.cxx: Attempt to fix a bug in
  SGSocket::read() affecting TCP style sockets.

2003-11-27 11:23  ehofman

* simgear/nasal/nasal.h: Revert an IRIX/O2 only setting again

2003-11-27 11:22  ehofman

* simgear/: compatibility/iomanip, nasal/nasal.h,
  structure/event_mgr.hxx: Portability fix

2003-11-26 15:42  ehofman

* simgear/nasal/.cvsignore: Make sure autogenerated files don't
  show up

2003-11-26 15:39  ehofman

* simgear/nasal/: lib.c, mathlib.c, parse.c: Portability fixes

2003-11-25 23:49  curt

* simgear/nasal/Makefile.am: rename libnasal.a to libsgnasal.a to
  match the existing SimGear library naming convention.

2003-11-25 22:45  ehofman

* simgear/compiler.h: MSVC > 6.0 fix

2003-11-25 22:26  curt

* configure.ac, simgear/Makefile.am, simgear/debug/debug_types.h,
  simgear/structure/event_mgr.cxx, simgear/structure/event_mgr.hxx:
  Nasal and event manager updates from Andy Ross.

2003-11-25 21:16  curt

* simgear/nasal/: Makefile.am, code.c, code.h, codegen.c, data.h,
  debug.c, gc.c, hash.c, lex.c, lib.c, mathlib.c, misc.c, nasal.h,
  parse.c, parse.h, string.c, vector.c: Version 1.3 of Andy Ross's
  "NASAL" embedded scripting language.

2003-11-24 18:41  david

* simgear/scene/: material/matlib.cxx, sky/cloud.cxx: Minor changes
  to logging messages (to STDOUT).

2003-11-23 11:04  ehofman

* simgear/scene/material/matlib.cxx: MacOS X fixes

2003-11-21 22:56  ehofman

* simgear/misc/sg_path.cxx: FIx a typo

2003-11-19 16:16  ehofman

* simgear/scene/model/model.cxx:
  Frederic Bouvier wrote:
  > When a c172 is on one machine, I only got segfault on an
  animation
  > not found. This animation is named ControlsGroup and I guess
  > that one object name referenced in this null animation no
  > longer exist in the .ac model. I say that because the
  > preceding one is analogous and works.
  >
  > So I would say the multiplayer works, except when there is a
  c172.

  It seems that this animation refers to 'PanelInstruments' that is
  a panel outside the model, so, when loaded with
  SGModelLib::load_model the object is not found in the model and
  there is a non tested read access through a null pointer in
  sgMakeAnimation.

2003-11-09 09:56  ehofman

* simgear/compatibility/sstream: Change istringstream back to a
  typedef so there is no need to redefine every class memeber.

2003-11-04 14:25  ehofman

* simgear/scene/model/animation.cxx: Save on a number of CPU costly
  strcmp calls when using the blend function

2003-10-22 21:21  curt

* Doxyfile, NEWS, configure.ac: Updates for the official 0.3.4
  version.

2003-10-20 21:53  ehofman

* simgear/compatibility/sstream: Shoot, I was trying to hunt down a
  bug that wasn't even caused by the sstream implementation! Back
  out some of the previous patches

2003-10-20 21:38  ehofman

* simgear/compatibility/sstream: And don't forget to free up the
  used memory.

2003-10-20 21:32  ehofman

* simgear/compatibility/sstream: Make it easy on myself (and make
  it work as a bonus)

2003-10-20 14:14  ehofman

* simgear/compatibility/sstream: Last fixes

2003-10-20 11:41  ehofman

* simgear/compatibility/sstream: Safety updates

2003-10-20 11:06  ehofman

* simgear/compatibility/: iomanip, iostream, sstream: fixes and
  updates for fgrun

2003-10-16 16:53  ehofman

* simgear/props/props_io.cxx: Fix a problem which was introduced in
  the previous patch

2003-10-16 14:51  ehofman

* simgear/: compiler.h, compatibility/IRIX,
  compatibility/Makefile.am, io/sg_socket.cxx,
  misc/tabbed_values_test.cxx, props/props.cxx, props/props_io.cxx,
  structure/exception.cxx: Fix a problem where older IRIX compilers
  needed a typecast for certain opperations

2003-10-15 22:15  curt

* Doxyfile, NEWS, SimGear.dsp, configure.ac, simgear/Makefile.am:
  Various updates for the upcoming 0.3.4 release.

2003-09-28 10:38  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Add the ability to set three levels of detail for static scenery
  using the property tree

2003-09-25 10:22  ehofman

* simgear/: Makefile.am, props/Makefile.am: Fix a dependency
  problem

2003-09-24 21:56  curt

* simgear/scene/tgdb/: obj.cxx, obj.hxx: A couple changes to allow
  separate handling of VASI/PAPI lights which generally are turned
  on all the time.

2003-09-24 21:06  ehofman

* simgear/structure/: event_mgr.cxx, event_mgr.hxx: Fix a problem
  where the compiler would mix up two function declarations because
  the one wich has SGSubsystem in it's options list expected a
  const SGSubsystem, but it was called with a plain SGSubsystem

2003-09-24 19:19  ehofman

* configure.ac, simgear/Makefile.am, simgear/misc/Makefile.am,
  simgear/misc/commands.cxx, simgear/misc/commands.hxx,
  simgear/misc/exception.cxx, simgear/misc/exception.hxx,
  simgear/props/condition.cxx, simgear/scene/material/matlib.cxx,
  simgear/scene/model/model.cxx, simgear/structure/.cvsignore,
  simgear/structure/Makefile.am, simgear/structure/callback.hxx,
  simgear/structure/commands.cxx, simgear/structure/commands.hxx,
  simgear/structure/event_mgr.cxx, simgear/structure/event_mgr.hxx,
  simgear/structure/exception.cxx, simgear/structure/exception.hxx,
  simgear/structure/subsystem_mgr.cxx,
  simgear/structure/subsystem_mgr.hxx, simgear/xml/easyxml.hxx:
  Move FGEventMgr and FGSubsystemMgr over to SimGear, add
  SGEventMgr to FlightGear's globals structre and some small code
  cleanups

2003-09-24 01:06  curt

* simgear/scene/: material/matlib.cxx, tgdb/obj.cxx,
  tgdb/pt_lights.hxx: Various tweaks to handling taxiway lighting.

2003-09-23 10:42  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Fix a problem where the
  upper cloud layers are drawn with the sun punch through value

2003-09-22 14:32  ehofman

* simgear/scene/sky/cloud.cxx: Fix a problem where the cloud layers
  suddenly change color when looking towards the sun

2003-09-17 19:59  ehofman

* simgear/scene/sky/: oursun.cxx, oursun.hxx: Initialize some
  variables before using them

2003-09-13 13:45  ehofman

* simgear/scene/model/model.cxx:
  Frederic Bouvier: This patch is there to correct a problem that
  prevent to load static objects when specifying a relative fg-root
  or a different, relative, fg-scenery. It appears that there is a
  mix between fg-root, fg-scenery and PLIB's model-dir.  It has
  been reported on the list that users are not able to see the
  buildings, especially those running the win32 builds because they
  run 'runfgfs.bat' that set FG_ROOT=./DATA.

  I decided not to use model-dir because it just add confusion and
  to build a valid path earlier.

2003-09-13 13:33  ehofman

* simgear/threads/SGThread.cxx: Add support for win32-pthreads in
  MSVC.NET

2003-09-12 23:24  ehofman

* simgear/screen/extensions.hxx: Fix a problem for systems with
  older headers

2003-09-09 16:33  ehofman

* simgear/scene/material/mat.cxx: Use default OpenGL material
  colors

2003-09-08 15:11  ehofman

* simgear/scene/material/mat.cxx: Change the defaults color
  specifications

2003-09-05 14:36  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Add a blend animation

2003-09-02 11:11  ehofman

* simgear/scene/sky/cloud.cxx: Update some light parameters

2003-08-31 11:23  ehofman

* simgear/scene/sky/cloud.cxx: Fix the box like cloud layer
  appearance

2003-08-31 05:12  curt

* simgear/scene/material/matlib.cxx: Fix some lighting values.

2003-08-29 09:35  ehofman

* simgear/: ephemeris/Makefile.am, io/Makefile.am,
  magvar/Makefile.am, misc/Makefile.am, props/Makefile.am,
  route/Makefile.am, screen/Makefile.am, xml/Makefile.am: New
  automake, new problems. Use $base_LIBS where $LIBS was
  automatically added before

2003-08-29 06:19  curt

* README.metakit, configure.ac: Remove metakit from src-libs,
  remove metakit check from configure script, remove
  README.metakit.

2003-08-22 18:58  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Actually use the calculated
  position in the layer list. This prevents the use of
  glDepthMask(). Clean up the code some.

2003-08-22 11:48  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Disable depth buffer writes
  while drawing the layers and some cosmetic updates

2003-08-22 10:07  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: A patch from Frederic
  Bouvier which performs z-buffer ordering of the cloud layers to
  prevent transparency problems with other (se mi) transparent
  objects. Good work Frederic!

2003-08-19 14:04  ehofman

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Adjust the layer span
  and amount of curving based on the eleveation of the layer

2003-08-19 04:08  curt

* simgear/: misc/texcoord.cxx, misc/texcoord.hxx,
  scene/tgdb/obj.cxx: Give calc_tex_coords() a more conformant
  name: sgCalcTexCoords()

2003-08-15 19:44  ehofman

* simgear/scene/sky/oursun.cxx: Don't use glGet because of
  performance issues

2003-08-15 19:19  ehofman

* simgear/scene/sky/oursun.cxx: Add support for NVidia cards with a
  broken OpenGL implementation

2003-08-14 14:32  ehofman

* simgear/scene/sky/oursun.cxx: A small update to the fog punch
  through code

2003-08-14 11:58  ehofman

* simgear/scene/sky/: oursun.cxx, oursun.hxx, sky.cxx: Adjust the
  fog punch through effect for oursun

2003-08-11 23:16  curt

* configure.ac, simgear/scene/material/matlib.cxx,
  simgear/scene/sky/dome.cxx, simgear/screen/GLBitmaps.cxx,
  simgear/screen/screen-dump.cxx: Remove extraneous/unneeded
  dependencies on glut.  SimGear should no longer have any glut
  dependies.

2003-08-11 21:42  curt

* simgear/screen/: extensions.cxx, extensions.hxx: Oops, it doesn't
  do much good to declare a function as "static" in the .hxx

2003-08-09 04:54  curt

* simgear/misc/sg_path.cxx: Only use the ";" delimiter under WIN32

2003-08-08 21:54  curt

* simgear/misc/: sg_path.cxx, sg_path.hxx: Add a routines that
  takes a search path (separated by sgSearchPathSep) and seperates
  them into a vector of strings which it then returns.

2003-08-07 14:31  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Add support for rgba
  textures

2003-08-05 16:45  curt

* configure.ac: Changed "GL/glut.h" to <GL/glut.h>

2003-08-04 19:29  ehofman

* configure.ac, simgear/scene/sky/Makefile.am: Put clouds3d back in
  the build proces after (hopefully) fixing all the build problems

2003-08-04 19:05  ehofman

* simgear/screen/extensions.cxx: Add a safety precausion

2003-08-04 14:54  ehofman

* simgear/screen/extensions.hxx: Add runtime detection of
  glActiveTextureARB

2003-08-04 14:07  ehofman

* simgear/scene/sky/cloud.cxx: Pre-initialize the variables driving
  the external force

2003-08-01 16:20  ehofman

* simgear/scene/sky/dome.cxx: Adjust the fog according to
  visibillity

2003-07-31 16:46  ehofman

* simgear/scene/sky/: cloud.cxx, cloud.hxx, sky.cxx, sky.hxx: Add
  cloud movement direction and speed

2003-07-31 11:04  ehofman

* simgear/scene/sky/: sky.cxx, sky.hxx: Keep the stack clean

2003-07-25 16:48  curt

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Actually commit the code changes which impliment a "scale"
  animation type.

2003-07-23 11:59  ehofman

* simgear/scene/model/animation.cxx: MSVC warning fixes

2003-07-21 10:37  ehofman

* simgear/scene/model/animation.cxx: Jim Wilson: Fixed texture
  translation so step and scroll values work with interpolation
  tables as well.  Moved step/scroll calculation to utility
  function to improve code readability.

2003-07-16 13:32  ehofman

* configure.ac, simgear/Makefile.am: Fix a typo

2003-07-13 14:34  ehofman

* configure.ac, simgear/Makefile.am: Don't bother other develoers
  with problems caused by MipsPro (version < 7.3) compilers

2003-07-12 11:18  ehofman

* configure.ac: Don't check for OpenGL libraries without at least
  including -lm

2003-07-11 19:50  curt

* simgear/screen/: texture.cxx, texture.hxx: Attempt to get these
  files back to a compilable state.

2003-07-11 12:55  ehofman

* simgear/screen/texture.cxx: Don't use floats where ints are more
  appropriate

2003-07-11 11:57  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Allow removing of the
  texture data after it is sent to OpenGL

2003-07-10 12:02  ehofman

* configure.ac, simgear/io/Makefile.am: Sync he configure script
  with that one from FLightGear by splitting the LIBS cariable into
  a base_LIBS, opengl_LIBS, network_LIBS and thread_LIBS variable

2003-07-10 11:49  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Jim Wilson: This update adds the ability to do multiple texture
  transforms (Steve B. thinks supporting them at the plib level
  would be inefficient, which is probably true).

  Removed units (e.g. "_m") from texture translation property and
  variable names since the texture translation values are
  dimensionless.

  Added the ability to specify a scroll factor for stepped texture
  animation that needs to scroll smoothly when approaching the step
  interval (e.g. odometer movement).

2003-07-10 11:14  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Don't delete the
  texture data after sending it to OpenGL.

2003-07-09 22:58  curt

* simgear/screen/: texture.cxx, texture.hxx: A couple more sanity
  checking tweaks for texture freeing.

2003-07-09 22:51  ehofman

* simgear/screen/texture.hxx: Prevent deleting the texture data for
  a second time in the destructor

2003-07-09 21:46  ehofman

* configure.ac: Fix a silly bug where is tested against the wrong
  variable

2003-07-09 16:46  ehofman

* simgear/scene/sky/Makefile.am: Leave the clouds3d commented out
  for now

2003-07-09 16:43  ehofman

* simgear/scene/sky/Makefile.am: A first attempt at making the
  clouds3d endian aware. Almost there.

2003-07-09 15:40  ehofman

* configure.ac: Back out a patch that never went in CVS ...

2003-07-07 13:27  ehofman

* configure.ac: Don't link agains OpenGL libraries when it's not
  needed

2003-07-06 19:13  ehofman

* configure.ac: Clean up, and make more flexible and prevent
  unnessecary library includes

2003-07-02 14:25  ehofman

* configure.ac: Solaris updates

2003-07-01 11:49  ehofman

* simgear/screen/: texture.cxx, texture.hxx: Add a function which
  might return whether a texture is in video memory, delete the
  texture buffer after sending it to OpenGL and comment out the
  set/get_pixel functions

2003-06-28 23:43  ehofman

* simgear/math/: fastmath.cxx, fastmath.hxx: MSVC fixes

2003-06-28 15:43  ehofman

* simgear/sound/sound.cxx: Put the refference to fast_log() back in
  after checkit it actually works as expected

2003-06-28 14:58  ehofman

* simgear/: math/fastmath.cxx, math/fastmath.hxx, sound/sound.cxx:
  Fix some problems

2003-06-28 14:06  ehofman

* simgear/: math/Makefile.am, math/fastmath.cxx, math/fastmath.hxx,
  sound/sound.cxx: Add some fast math functions

2003-06-27 23:36  ehofman

* simgear/scene/sky/oursun.cxx: Do some math omtimizations as
  pointed out by Norman

2003-06-26 19:13  curt

* simgear/ephemeris/ephemeris.cxx: Explicitely initialize planets.

2003-06-24 10:22  ehofman

* simgear/screen/extensions.hxx: Use the dlsym based approach on
  all unices and depreciate the glXGetProcAddressARB function

2003-06-22 13:49  ehofman

* simgear/screen/: extensions.cxx, extensions.hxx: cygwin and mingw
  fixes

2003-06-20 22:05  ehofman

* simgear/screen/extensions.hxx: Cross platform
  fixsimgear/screen/extensions.hxx

2003-06-20 21:44  ehofman

* simgear/sound/soundmgr.cxx: Increase the maximum number of
  simultanious audio streams to the maximum defined by plib

2003-06-20 21:32  ehofman

* simgear/screen/extensions.cxx: Mac OS X fix

2003-06-19 09:40  ehofman

* simgear/screen/: extensions.cxx, extensions.hxx: MSVC and Apple
  OS X fixes

2003-06-18 11:06  ehofman

* simgear/screen/: Makefile.am, extensions.cxx, extensions.hxx: Add
  our own function to check whether a certain OpenGL extension is
  supported

2003-06-17 18:55  ehofman

* simgear/screen/: Makefile.am, extensions.hxx: Add an OpenGL
  extension query function which should be cross platform

2003-06-13 21:56  ehofman

* simgear/scene/sky/oursun.cxx: Don't make the sun imune for fog

2003-06-11 20:55  curt

* DoxygenMain.cxx, simgear/scene/sky/cloud.hxx,
  simgear/scene/sky/sky.cxx, simgear/scene/sky/sky.hxx: - Tweaks to
  doxygen main page.  - Added documentation for SGCloudLayer -
  Updated the SGSky interface a bit to make it more sensible,
  flexible,   and generic.  This requires a code tweak on the
  FlightGear side as well.

2003-06-09 22:19  curt

* simgear/: scene/sky/dome.cxx, timing/sg_time.cxx,
  timing/sg_time.hxx: Make sky dome scaling values sensible (i.e.
  the sky dome will now fill up   the dimensions provided.)  We
  draw the sky dome before everything else   and draw it with depth
  buffer off so it really doesn't matter, but it just	makes a
  little more sense this way.  Updated a few doxygen comments.

2003-06-09 11:11  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx: Add the
  ability to include stepped texture translations for things like
  digital displays in 3D model animation.

2003-06-08 15:19  ehofman

* simgear/scene/model/: animation.cxx, animation.hxx, model.cxx:
  Jim Wilson: 1. Added support for defining arbitrary rotation axes
  using (x1,y1,z1), (x2,y2,z2).  The center is calculated
  automatically (midpoint on line) or you may specify an alternate
  "center" using the current scheme.  This makes it about  100
  times easier to animate flaps, ailerons, etc.

  2. Added support for plib's ssgTexTrans.  This will allow more
  sophisticated 3D instrument features by allowing the texture
  mapping itself to be animated. Included function for "texrotate"
  and "textranslate".  They configure the same as the geometry,
  except the arbitrary axis definition is not necessary (textures
  are flat).

2003-06-07 12:35  ehofman

* simgear/scene/sky/oursun.cxx: Remove some now depreciated
  lighting code

2003-06-03 21:37  ehofman

* simgear/sound/soundmgr.cxx: Remove an unused variable

2003-06-03 21:35  ehofman

* simgear/sound/: sound.hxx, soundmgr.hxx: Add some more
  descriptive comments

2003-06-03 20:48  curt

* Doxyfile, DoxygenMain.cxx, Makefile.am, NEWS, configure.ac:
  Various 0.3.3 last minute tweaks.

2003-06-03 20:32  curt

* Doxyfile, DoxygenMain.cxx, simgear/math/sg_geodesy.hxx,
  simgear/screen/texture.cxx, simgear/screen/texture.hxx,
  simgear/timing/geocoord.cxx, simgear/timing/geocoord.h,
  simgear/timing/sg_time.cxx, simgear/timing/sg_time.hxx,
  simgear/timing/timezone.cxx, simgear/timing/timezone.h: Various
  documentation tweaks and additions.

2003-06-03 20:22  curt

* simgear/sound/: sound.hxx, soundmgr.hxx: Tweaks to doxygen
  comments.

2003-06-03 15:30  ehofman

* simgear/scene/tgdb/obj.cxx: compiler fixes

2003-06-02 22:11  curt

* SimGear.dsp, configure.ac: Updated dsp/dsw files for MSVC.

2003-06-02 21:58  curt

* Doxyfile, NEWS, configure.ac: Tweaks for the 0.3.2 release.

2003-06-02 17:23  curt

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Fix a bug in cloud
  texture state loading which caused the cloud textures to be
  loaded 5 times! for a lot of wasted texture RAM.  Thanks to Erik
  H. for noticing the problem.

2003-05-30 18:41  curt

* Doxyfile, DoxygenMain.cxx, Makefile.am: Working on adding a top
  leve Doxygen docs introduction.

2003-05-30 17:27  curt

* simgear/scene/tgdb/: obj.cxx, userdata.cxx, userdata.hxx: Clean
  up a class renaming mistake.

2003-05-30 17:16  curt

* configure.ac, simgear/Makefile.am, simgear/scene/Makefile.am,
  simgear/scene/sky/Makefile.am, simgear/scene/sky/sky.hxx: Move
  simgear/sky/ -> simgear/scene/sky/ as part of the consolidation
  of graphics code.

2003-05-30 16:59  curt

* Doxyfile, simgear/io/sg_serial.hxx, simgear/serial/serial.cxx,
  simgear/serial/serial.hxx, simgear/serial/testserial.cxx: Rename
  FGSerialPort to SGSerialPort.

2003-05-28 23:00  curt

* simgear/scene/tgdb/obj.cxx: Fix a very small oops.

2003-05-28 22:59  curt

* simgear/scene/tgdb/: Makefile.am, obj.cxx, obj.hxx: Moved low
  level "tg" format object loader code over to SimGear.

2003-05-28 22:25  curt

* simgear/scene/tgdb/: Makefile.am, userdata.cxx, userdata.hxx:
  Moved random ground cover object management code over to simgear.

2003-05-28 21:02  curt

* simgear/scene/tgdb/: leaf.cxx, leaf.hxx: Make gen_surface_points
  accessible through the API.

2003-05-21 19:17  ehofman

* simgear/: screen/colors.hxx, scene/sky/dome.cxx,
  scene/sky/oursun.cxx: Updates for a better sunrise/sunset effect

2003-05-19 17:40  ehofman

* simgear/timing/: geocoord.h, sg_time.cxx, sg_time.hxx,
  timezone.cxx, timezone.h: Convert char* to string to prevent
  stdup/malloc/free

2003-05-17 14:43  ehofman

* simgear/screen/: Makefile.am, colors.hxx: Add a gamma correction
  function

2003-05-16 18:32  ehofman

* simgear/scene/sky/: oursun.cxx, sky.cxx, sky.hxx: Adjust
  sunset/sun rise color scheme

2003-05-16 15:22  curt

* simgear/scene/material/mat.hxx: Forgot to #include <vector>

2003-05-16 13:40  curt

* simgear/scene/material/: mat.cxx, mat.hxx, matmodel.cxx,
  matmodel.hxx: I missed committing t hese changes yesterday.

2003-05-16 10:51  ehofman

* simgear/scene/material/: mat.cxx, mat.hxx, matmodel.cxx,
  matmodel.hxx: A patch from Frederic Bouvier to correct a naming
  problem caused bu Curts work. This gets FlightGear/SimGear
  compiling again

2003-05-15 23:35  curt

* simgear/scene/model/: Makefile.am, loader.cxx, loader.hxx,
  location.cxx, model.cxx, modellib.cxx, modellib.hxx,
  placement.cxx: Various code massaging.

2003-05-15 18:19  curt

* simgear/scene/material/: Makefile.am, mat.cxx, mat.hxx,
  matmodel.cxx, matmodel.hxx, matobj.cxx, matobj.hxx: Rename matobj
  -> matmodel.

2003-05-15 17:08  curt

* simgear/scene/material/: Makefile.am, mat.cxx, mat.hxx,
  matobj.cxx, matobj.hxx: Separate out the
  SGMaterial::Object{,Group} code into it's own source file.

2003-05-14 22:36  ehofman

* configure.ac, simgear/scene/Makefile.am,
  simgear/scene/tgdb/.cvsignore: Add some missing requirements

2003-05-14 21:31  curt

* simgear/scene/tgdb/: apt_signs.cxx, apt_signs.hxx, pt_lights.cxx,
  pt_lights.hxx: Just a bit of renaming.

2003-05-14 21:20  curt

* simgear/scene/tgdb/: Makefile.am, apt_signs.cxx, apt_signs.hxx,
  leaf.cxx, leaf.hxx, pt_lights.cxx, pt_lights.hxx: Moved some low
  level scene graph construction code over to simgear/scene/tgdb/

2003-05-14 20:27  curt

* simgear/scene/material/: matlib.cxx, matlib.hxx: Removed global
  instance of the material manager.  Each application will need to
  create it's own instance itself.

2003-05-13 21:05  curt

* simgear/scene/material/: mat.cxx, mat.hxx, matlib.cxx,
  matlib.hxx: Removed non-textured and flat shaded support because
  it really clutters up the API and I don't believe we'd have any
  hope of running at any kind of reasonable frame rates on a
  non-hardware-3d accelerated box these days anyway.

2003-05-13 21:00  ehofman

* simgear/scene/sky/: oursun.hxx, sky.hxx: Make the current color
  of the sun available

2003-05-13 19:14  ehofman

* simgear/scene/sky/cloud.cxx: Improve speed and readabillity

2003-05-13 14:11  ehofman

* simgear/scene/sky/cloud.cxx: Oops, make the cloud layer look more
  like an inverted bowl than like a sombrero

2003-05-13 12:21  david

* simgear/scene/material/.cvsignore: Added generated files.

2003-05-13 12:21  david

* simgear/props/.cvsignore: Added props_test program.

2003-05-13 05:18  curt

* simgear/: props/condition.cxx, props/condition.hxx,
  scene/material/mat.cxx, scene/material/mat.hxx,
  scene/material/matlib.cxx, scene/material/matlib.hxx,
  scene/model/animation.cxx, scene/model/animation.hxx,
  scene/model/loader.cxx, scene/model/loader.hxx,
  scene/model/location.cxx, scene/model/location.hxx,
  scene/model/model.cxx, scene/model/model.hxx,
  scene/model/placement.cxx, scene/model/placement.hxx,
  scene/sky/sky.cxx, sound/sound.cxx, sound/sound.hxx,
  threads/SGQueue.hxx, threads/SGThread.cxx, threads/SGThread.hxx:
  Cosmetic changes for new code moved into simgear to make the
  naming scheme better follow simgear conventions.

2003-05-12 23:30  curt

* configure.ac, simgear/scene/Makefile.am,
  simgear/scene/material/Makefile.am,
  simgear/scene/material/mat.cxx, simgear/scene/material/mat.hxx,
  simgear/scene/material/matlib.cxx,
  simgear/scene/material/matlib.hxx: Moved
  fgfs_src/Object/newmat.[ch]xx and fgfs_src/Object/matlib.[ch]xx
  into simgear/scene/material/

2003-05-12 00:29  ehofman

* simgear/scene/sky/: cloud.cxx, cloud.hxx: simgear/sky/

2003-05-11 22:54  ehofman

* simgear/scene/model/: model.cxx, placement.cxx: Cygwin build
  fixes

2003-05-09 22:19  curt

* simgear/scene/model/: Makefile.am, loader.cxx, loader.hxx,
  model.cxx, model.hxx: Moved loader.[ch]xx and model.[ch]xx from
  fgfs/src/Model/ to simgear/scene/model.

2003-05-09 21:37  curt

* simgear/scene/model/: Makefile.am, animation.hxx, location.hxx:
  Minor tweaks and clean ups.

2003-05-09 21:37  curt

* simgear/scene/model/: placement.cxx, placement.hxx: Moved to
  SimGear from FlightGear/src/Model/

2003-05-09 21:36  curt

* simgear/sound/: sound.cxx, sound.hxx, soundmgr.cxx, soundmgr.hxx:
  Added "SG" prefixes to match other SimGear classes.

2003-05-09 19:29  ehofman

* configure.ac, simgear/Makefile.am,
  simgear/compatibility/Makefile.am, simgear/compatibility/new,
  simgear/sound/.cvsignore, simgear/sound/Makefile.am,
  simgear/sound/sound.cxx, simgear/sound/sound.hxx,
  simgear/sound/soundmgr.cxx, simgear/sound/soundmgr.hxx: Move the
  FlightGear sound code over to SimGear

2003-05-08 23:42  ehofman

* simgear/: props/.cvsignore, scene/.cvsignore,
  scene/model/.cvsignore: ignore certain files for CVS commits

2003-05-08 18:32  curt

* simgear/scene/model/: Makefile.am, animation.cxx, animation.hxx:
  Split out animation code from fgfs-src/Model/model.[ch]xx and
  move it over here.

2003-05-07 03:59  curt

* simgear/props/condition.cxx: Removed some extraneous debugging
  output.  Hey, this one was subtle.  Due to const/no-const
  variants of the prop->getNode() method, the 2nd bool argument was
  getting promoted to an int so it would match a method with a
  const SGPropertyNode * as the first argument.  But that made it
  match the wrong version of prop->getNode() and things were
  failing.  Yikes!  This is one I could have stared at for days to
  figure out so I'm glad I caught on to the problem. :-)

2003-05-07 01:40  curt

* configure.ac, simgear/Makefile.am, simgear/misc/Makefile.am,
  simgear/misc/commands.cxx, simgear/misc/commands.hxx,
  simgear/misc/props.cxx, simgear/misc/props.hxx,
  simgear/misc/props_io.cxx, simgear/misc/props_io.hxx,
  simgear/misc/props_test.cxx, simgear/props/Makefile.am,
  simgear/props/condition.cxx, simgear/props/condition.hxx,
  simgear/props/props.cxx, simgear/props/props.hxx,
  simgear/props/props_io.cxx, simgear/props/props_io.hxx,
  simgear/props/props_test.cxx, simgear/scene/Makefile.am,
  simgear/scene/model/Makefile.am,
  simgear/scene/model/location.cxx,
  simgear/scene/model/location.hxx: - Moved some property specific
  code into simgear/props/ - Split out the condition code from
  fgfs/src/Main/fg_props and put it   in it's own source file in
  simgear/props/ - Created a scene subdirectory for scenery, model,
  and material property   related code.  - Moved location.[ch]xx
  into simgear/scene/model/ - The location and condition code had
  dependencies on flightgear's global	state (all the globals->
  stuff, the flightgear property tree, etc.)  SimGear	code can't
  depend on it so that data has to be passed as parameters to the
  functions/methods/constructors.

2003-04-14 19:58  curt

* simgear/math/: sg_geodesy.cxx, sg_geodesy.hxx: Pass doubles as
  const ref's to save a couble bytes of stack space and presumably
  a tiny bit of function call time.

2003-04-13 23:16  ehofman

* simgear/screen/texture.cxx: Fixup for windows machines

2003-04-12 11:42  ehofman

* simgear/: misc/Makefile.am, screen/Makefile.am, misc/colours.h,
  screen/colours.h: Move the texture object to the screen directory
  for the sake of library dependencies

2003-04-12 11:29  ehofman

* simgear/: misc/Makefile.am, screen/Makefile.am,
  screen/texture.cxx, screen/texture.hxx, misc/texture.cxx,
  misc/texture.hxx: Move the texture object to the screen directory
  for the sake of library dependencies

2003-04-11 14:51  ehofman

* configure.ac: remove a refference to a directory that doesn't
  exists yet

2003-04-10 23:38  ehofman

* simgear/Makefile.am: remove a depreciated file

2003-04-10 23:37  ehofman

* configure.ac: Remove a depreciated file

2003-04-10 11:31  ehofman

* configure.ac: a small update

2003-04-10 11:27  ehofman

* configure.ac, simgear/sg_zlib.h: remove a depreciated file

2003-04-10 11:26  ehofman

* simgear/misc/: texture.cxx, texture.hxx: get rid of malloc()

2003-04-10 10:01  ehofman

* simgear/: math/interpolater.cxx, misc/texture.cxx,
  misc/texture.hxx: Depreciate a wrapper header file

2003-04-09 22:59  ehofman

* simgear/sg_zlib.h: Update a SimGear/FlightGear inconsistancy

2003-04-09 22:33  ehofman

* configure.ac: Fix a booboo

2003-04-09 22:29  ehofman

* simgear/scene/sky/Makefile.am: fix a booboo

2003-04-09 22:26  ehofman

* simgear/scene/sky/: dome.cxx, dome.hxx: fix a booboo

2003-04-09 22:24  ehofman

* configure.ac, simgear/sg_zlib.h, simgear/misc/Makefile.am,
  simgear/misc/colours.h, simgear/misc/texture.cxx,
  simgear/misc/texture.hxx, simgear/scene/sky/Makefile.am,
  simgear/scene/sky/dome.cxx, simgear/scene/sky/dome.hxx: Add out
  own texture object

2003-04-05 05:32  david

* simgear/scene/sky/: cloud.cxx, cloud.hxx: Use "coverage" instead
  of "type".

  Rename "mostly-cloudy" to "broken" and "mostly-sunny" to
  "scattered", to follow standard weather terminology.

  Add "few".

2003-03-22 11:34  ehofman

* simgear/debug/debug_types.h: Make debug levels an integer

2003-03-20 13:14  ehofman

* simgear/debug/debug_types.h: add Air Traffic Controll to the list
  of possible debug sections

2003-03-19 17:16  ehofman

* simgear/compatibility/: Makefile.am, fstream, iomanip, iostream,
  sstream: improved STL compatibility header files

2003-03-10 17:58  curt

* simgear/scene/sky/Makefile.am: Remove 3d clouds from the default
  build.  These can still be built manually if desired, no code is
  being removed.

2003-03-02 17:35  david

* simgear/misc/tabbed_values.cxx: Patch from Frederic Bouvier:

  I am trying to compile tabbed_values.cxx and found that it
  requires assert.h to compile with MSVC (on Linux, it must be
  included indirectly).  There is a patch below

2003-03-02 17:34  david

* simgear/misc/.cvsignore: Added tabbed_test program.

2003-03-02 15:52  david

* simgear/: io/sg_binobj.cxx, math/linintp2.inl, math/sphrintp.inl,
  misc/Makefile.am, misc/tabbed_values_test.cxx, serial/serial.cxx,
  scene/sky/oursun.cxx, scene/sky/sphere.cxx, scene/sky/stars.cxx,
  timing/sg_time.cxx: Patches from Erik Hofman (for Irix? I've lost
  the original message).

2003-02-28 02:02  curt

* simgear/misc/: sg_path.cxx, sg_path.hxx: Bernie Bright:

  Could the file(), dir(), base() and extension() functions be made
  const member functions.  As it stands they cannot be applied to
  const reference/pointer values which limits their usefulness.

  Curt:

  Yes.

2003-02-26 20:50  curt

* simgear/misc/: sg_path.cxx, sg_path.hxx: Add some convenience
  functions to the SGPath function.

2003-02-15 19:53  curt

* simgear/misc/sg_path.cxx: Oops, missed this the first time.

2003-02-15 19:43  curt

* simgear/misc/: Makefile.am, tabbed_values.cxx, tabbed_values.hxx,
  tabbed_values_test.cxx: James Turner:

  - added a new class in simgear/misc, SGTabbedValues, which parses
  a   null-terminated string of data separated by tabs into fields,
  and	supports safe conversion into various other datatypes.

2003-02-15 19:43  curt

* simgear/misc/sg_path.hxx: James Turner:

  - added a 'filename' helper to SGPath, corresponding to the
  'directory' helper but returning just the filename portion of the
  name.

2003-02-07 18:35  curt

* configure.ac: Fix a typo in the FreeBSD support section.

2003-01-23 22:59  curt

* configure.ac, simgear/scene/sky/cloud.cxx: Misc. tweaks that have
  been laying around.

2003-01-23 16:16  curt

* configure.ac: FreeBSD pthread and malloc fixes.

2003-01-02 21:11  curt

* simgear/compatibility/Makefile.am: Some how the contents of this
  file got doubled up ...

2002-12-31 19:03  david

* simgear/compatibility/: fstream, iostream, istream, streambuf,
  strstream: Patches from Erik Hofman for SGI compatibility:

  Some more cmall changes to the SimGear header files and removed
  the SG_HAVE_NATIVE_SGI_COMPILERS dependancies from FlightGear.

  I've added a separate JSBSim patch for the JSBSim source tree.

2002-12-31 15:47  david

* simgear/: bucket/newbucket.hxx, compatibility/Makefile.am,
  compatibility/iostream, compatibility/istream,
  compatibility/streambuf, debug/logstream.hxx,
  ephemeris/stardata.cxx, io/decode_binobj.cxx, io/lowtest.cxx,
  io/sg_binobj.cxx, io/socktest.cxx, io/tcp_client.cxx,
  io/tcp_server.cxx, math/point3d.hxx, math/sg_geodesy.cxx,
  misc/props.hxx, misc/props_io.cxx, misc/props_io.hxx,
  misc/props_test.cxx, misc/sgstream.hxx, misc/zfstream.hxx,
  route/routetest.cxx, route/waytest.cxx, serial/serial.cxx,
  serial/testserial.cxx, scene/sky/sphere.cxx, scene/sky/stars.cxx,
  xml/easyxml.cxx, xml/easyxml.hxx: Patch from Erik Hofman:

  This patch adds some more functionality to the header files and
  removes *all* dependencies on SG_HAVE_NATIVE_SGI_COMPILERS in the
  SimGear code!

  [dpm: I had to add some additional SG_USING_STD declarations to
  make it compile]

2002-12-30 23:33  david

* simgear/: Makefile.am, compatibility/.cvsignore,
  compatibility/Makefile.am: Added missing Makefile.am from Erik
  Hofman.

2002-12-30 22:33  david

* configure.ac, simgear/compiler.h, simgear/compatibility/README,
  simgear/compatibility/cassert, simgear/compatibility/cctype,
  simgear/compatibility/cerrno, simgear/compatibility/cfloat,
  simgear/compatibility/climits, simgear/compatibility/clocale,
  simgear/compatibility/cmath, simgear/compatibility/csetjmp,
  simgear/compatibility/csignal, simgear/compatibility/cstdarg,
  simgear/compatibility/cstddef, simgear/compatibility/cstdio,
  simgear/compatibility/cstdlib, simgear/compatibility/cstring,
  simgear/compatibility/ctime, simgear/compatibility/cwchar,
  simgear/compatibility/cwctype, simgear/compatibility/fstream,
  simgear/compatibility/iomanip, simgear/compatibility/iostream,
  simgear/compatibility/istream, simgear/compatibility/iterator,
  simgear/compatibility/streambuf, simgear/compatibility/strstream:
  IRIX/MipsPro patches from Erik Hofman:

  This patch adds some missing default ISO C++ headers to SimGear,
  especially usefull for Irix/MipsPro. To the best of my knowledge
  this only affects the Irix/MipsPro compiler combination although
  it might be usefull for others as well.

  This patch does not yet remove any compiler specific code, but
  that would be my next task. The powerfull macros as defined in
  the compiler.h file allowed me to split it up into multiple
  patches.

2002-12-21 14:40  david

* configure.ac: Some trivial typo corrections.

2002-12-18 22:54  curt

* configure.ac: Mingwin fix from Norman Vine.

2002-12-11 20:57  curt

* configure.ac: Remove -lmk4 from the LIBS after the version check
  is complete.

2002-12-10 21:54  curt

* configure.ac: More tweaks to the configure script.

2002-12-10 20:12  curt

* configure.ac, simgear/debug/logstream.hxx,
  simgear/io/Makefile.am, simgear/screen/GLBitmaps.cxx,
  simgear/screen/Makefile.am, simgear/screen/screen-dump.cxx,
  simgear/scene/sky/dome.cxx: - Refactoring configure.ac a bit to
  use $host (please test on your platform) - Use include GLUT_H
  instead of refering to the file directly since Mac
  unfortunately chose to put this in GLUT/glut.h :-(

2002-12-09 23:36  curt

* configure.ac: James Turner:

  I've had to hack Simgear's configure.ac quite a bit [for Mac OS
  X], using the Plib one as a reference.  The basic construct (a
  big switch statement based on the target type) is nice, I think,
  since it moves lots of IRIX / cygwin / OS-X specialties out of
  the way cleanly. Much more re-factoring of the current tests in
  configure is possible if people are able to test.

2002-12-04 20:47  curt

* Doxyfile, NEWS: Updated for 0.3.1

2002-12-04 20:46  curt

* SimGear.dsp, configure.ac: Fixes for 0.3.1 release.

2002-12-03 19:27  curt

* NEWS: Updated for 0.3.0 release.

2002-12-03 19:19  curt

* Doxyfile: Updated version number

2002-12-03 14:13  curt

* simgear/: bucket/newbucket.cxx, io/sg_binobj.cxx: #include
  <simgear_config.h> as necessary.

2002-12-02 23:12  curt

* configure.ac, simgear/compiler.h, simgear/math/sg_types.hxx:
  NOMINMAX fix ...

2002-11-17 01:34  david

* simgear/compiler.h: MS patch from Norm Vine to fix min/max macro
  defs.

2002-11-11 15:40  david

* simgear/misc/props_io.cxx: Allow 'include' attribute on root
  PropertyList element.

2002-10-26 03:18  david

* simgear/misc/: commands.cxx, commands.hxx: Simplified the
  command-manager interface.

2002-10-02 18:03  curt

* simgear/scene/sky/cloud.cxx: ref() the cloud states before use so
  that if we are dyanmically creating and deleting cloud layers we
  don't inadvertantly delete a cloud state.

2002-09-18 22:38  curt

* Makefile.am, SimGear.dsp, simgear/Makefile.am: Tweaks to build
  system following removal of interpreter subdir.

2002-09-18 22:29  david

* configure.ac: Removed simgear/interpreter directory; most likely
  we will use the new PSL interpreted language in plib.

2002-09-18 16:24  curt

* configure.ac: Norman's most recent 3d clouds code tweaks.

2002-09-14 18:06  david

* .cvsignore: Added autom4te.cache.

2002-09-14 18:05  david

* simgear/.cvsignore: Added stamp-h1.

2002-09-14 18:05  david

* configure.ac: Removed /usr/local/include (as with plib).

2002-09-14 01:19  curt

* configure.ac: Upgrade the version number.

2002-09-14 00:51  curt

* configure.ac, simgear/scene/sky/Makefile.am: Added
  simgear/sky/clouds3d to the build system.

2002-09-07 04:58  curt

* .cvsignore, AUTHORS, COPYING, ChangeLog, Doxyfile, Makefile.am,
  NEWS, README, README.MSVC, README.metakit, README.plib,
  README.zlib, SimGear.dsp, SimGear.dsw, SimGear.spec.in, TODO,
  Thanks, acinclude.m4, am2dsp.cfg, autogen.sh, configure.ac,
  simgear/.cvsignore, simgear/Makefile.am, simgear/compiler.h,
  simgear/constants.h, simgear/sg_inlines.h, simgear/sg_traits.hxx,
  simgear/sg_zlib.h, simgear/simgear_config.h.vc5,
  simgear/version.h.in, simgear/bucket/.cvsignore,
  simgear/bucket/Makefile.am, simgear/bucket/newbucket.cxx,
  simgear/bucket/newbucket.hxx, simgear/debug/.cvsignore,
  simgear/debug/Makefile.am, simgear/debug/debug_types.h,
  simgear/debug/logstream.cxx, simgear/debug/logstream.hxx,
  simgear/debug/logtest.cxx, simgear/magvar/.cvsignore,
  simgear/magvar/Makefile.am, simgear/magvar/coremag.cxx,
  simgear/magvar/coremag.hxx, simgear/magvar/magvar.cxx,
  simgear/magvar/magvar.hxx, simgear/magvar/testmagvar.cxx,
  simgear/math/.cvsignore, simgear/math/Makefile.am,
  simgear/math/interpolater.cxx, simgear/math/interpolater.hxx,
  simgear/math/leastsqs.cxx, simgear/math/leastsqs.hxx,
  simgear/math/linintp2.h, simgear/math/linintp2.inl,
  simgear/math/localconsts.hxx, simgear/math/point3d.hxx,
  simgear/math/polar3d.cxx, simgear/math/polar3d.hxx,
  simgear/math/sg_geodesy.cxx, simgear/math/sg_geodesy.hxx,
  simgear/math/sg_memory.h, simgear/math/sg_random.c,
  simgear/math/sg_random.h, simgear/math/sg_types.hxx,
  simgear/math/sphrintp.h, simgear/math/sphrintp.inl,
  simgear/math/vector.cxx, simgear/math/vector.hxx,
  simgear/misc/.cvsignore, simgear/misc/Makefile.am,
  simgear/misc/commands.cxx, simgear/misc/commands.hxx,
  simgear/misc/exception.cxx, simgear/misc/exception.hxx,
  simgear/misc/props.cxx, simgear/misc/props.hxx,
  simgear/misc/props_io.cxx, simgear/misc/props_io.hxx,
  simgear/misc/props_test.cxx, simgear/misc/sg_path.cxx,
  simgear/misc/sg_path.hxx, simgear/misc/sgstream.cxx,
  simgear/misc/sgstream.hxx, simgear/misc/stopwatch.hxx,
  simgear/misc/strutils.cxx, simgear/misc/strutils.hxx,
  simgear/misc/texcoord.cxx, simgear/misc/texcoord.hxx,
  simgear/misc/zfstream.cxx, simgear/misc/zfstream.hxx,
  simgear/screen/.cvsignore, simgear/screen/GLBitmaps.cxx,
  simgear/screen/GLBitmaps.h, simgear/screen/Makefile.am,
  simgear/screen/jpgfactory.cxx, simgear/screen/jpgfactory.hxx,
  simgear/screen/screen-dump.cxx, simgear/screen/screen-dump.hxx,
  simgear/screen/tr.cxx, simgear/screen/tr.h,
  simgear/screen/win32-printer.h, simgear/serial/.cvsignore,
  simgear/serial/Makefile.am, simgear/serial/serial.cxx,
  simgear/serial/serial.hxx, simgear/serial/testserial.cxx,
  simgear/timing/.cvsignore, simgear/timing/Makefile.am,
  simgear/timing/geocoord.cxx, simgear/timing/geocoord.h,
  simgear/timing/lowleveltime.cxx, simgear/timing/lowleveltime.h,
  simgear/timing/sg_time.cxx, simgear/timing/sg_time.hxx,
  simgear/timing/timestamp.cxx, simgear/timing/timestamp.hxx,
  simgear/timing/timezone.cxx, simgear/timing/timezone.h,
  simgear/ephemeris/.cvsignore, simgear/ephemeris/Makefile.am,
  simgear/ephemeris/celestialBody.cxx,
  simgear/ephemeris/celestialBody.hxx,
  simgear/ephemeris/ephemeris.cxx, simgear/ephemeris/ephemeris.hxx,
  simgear/ephemeris/jupiter.cxx, simgear/ephemeris/jupiter.hxx,
  simgear/ephemeris/mars.cxx, simgear/ephemeris/mars.hxx,
  simgear/ephemeris/mercury.cxx, simgear/ephemeris/mercury.hxx,
  simgear/ephemeris/moonpos.cxx, simgear/ephemeris/moonpos.hxx,
  simgear/ephemeris/neptune.cxx, simgear/ephemeris/neptune.hxx,
  simgear/ephemeris/pluto.hxx, simgear/ephemeris/saturn.cxx,
  simgear/ephemeris/saturn.hxx, simgear/ephemeris/star.cxx,
  simgear/ephemeris/star.hxx, simgear/ephemeris/stardata.cxx,
  simgear/ephemeris/stardata.hxx, simgear/ephemeris/uranus.cxx,
  simgear/ephemeris/uranus.hxx, simgear/ephemeris/venus.cxx,
  simgear/ephemeris/venus.hxx, simgear/io/.cvsignore,
  simgear/io/Makefile.am, simgear/io/decode_binobj.cxx,
  simgear/io/iochannel.cxx, simgear/io/iochannel.hxx,
  simgear/io/lowlevel.cxx, simgear/io/lowlevel.hxx,
  simgear/io/lowtest.cxx, simgear/io/sg_binobj.cxx,
  simgear/io/sg_binobj.hxx, simgear/io/sg_file.cxx,
  simgear/io/sg_file.hxx, simgear/io/sg_serial.cxx,
  simgear/io/sg_serial.hxx, simgear/io/sg_socket.cxx,
  simgear/io/sg_socket.hxx, simgear/io/sg_socket_udp.cxx,
  simgear/io/sg_socket_udp.hxx, simgear/io/socktest.cxx,
  simgear/io/tcp_client.cxx, simgear/io/tcp_server.cxx,
  simgear/route/.cvsignore, simgear/route/Makefile.am,
  simgear/route/route.cxx, simgear/route/route.hxx,
  simgear/route/routetest.cxx, simgear/route/waypoint.cxx,
  simgear/route/waypoint.hxx, simgear/route/waytest.cxx,
  simgear/scene/sky/.cvsignore, simgear/scene/sky/Makefile.am,
  simgear/scene/sky/cloud.cxx, simgear/scene/sky/cloud.hxx,
  simgear/scene/sky/dome.cxx, simgear/scene/sky/dome.hxx,
  simgear/scene/sky/moon.cxx, simgear/scene/sky/moon.hxx,
  simgear/scene/sky/oursun.cxx, simgear/scene/sky/oursun.hxx,
  simgear/scene/sky/sky.cxx, simgear/scene/sky/sky.hxx,
  simgear/scene/sky/sphere.cxx, simgear/scene/sky/sphere.hxx,
  simgear/scene/sky/stars.cxx, simgear/scene/sky/stars.hxx,
  simgear/threads/.cvsignore, simgear/threads/Makefile.am,
  simgear/threads/SGGuard.hxx, simgear/threads/SGQueue.hxx,
  simgear/threads/SGThread.cxx, simgear/threads/SGThread.hxx,
  simgear/xml/.cvsignore, simgear/xml/Makefile.am,
  simgear/xml/asciitab.h, simgear/xml/easyxml.cxx,
  simgear/xml/easyxml.hxx, simgear/xml/hashtable.c,
  simgear/xml/hashtable.h, simgear/xml/iasciitab.h,
  simgear/xml/latin1tab.h, simgear/xml/nametab.h,
  simgear/xml/sample.xml, simgear/xml/testEasyXML.cxx,
  simgear/xml/utf8tab.h, simgear/xml/xmldef.h,
  simgear/xml/xmlparse.c, simgear/xml/xmlparse.h,
  simgear/xml/xmlrole.c, simgear/xml/xmlrole.h,
  simgear/xml/xmltok.c, simgear/xml/xmltok.h,
  simgear/xml/xmltok_impl.c, simgear/xml/xmltok_impl.h,
  simgear/xml/xmltok_ns.c: Initial revision

2002-09-07 04:58  curt

* .cvsignore, AUTHORS, COPYING, ChangeLog, Doxyfile, Makefile.am,
  NEWS, README, README.MSVC, README.metakit, README.plib,
  README.zlib, SimGear.dsp, SimGear.dsw, SimGear.spec.in, TODO,
  Thanks, acinclude.m4, am2dsp.cfg, autogen.sh, configure.ac,
  simgear/.cvsignore, simgear/Makefile.am, simgear/compiler.h,
  simgear/constants.h, simgear/sg_inlines.h, simgear/sg_traits.hxx,
  simgear/sg_zlib.h, simgear/simgear_config.h.vc5,
  simgear/version.h.in, simgear/bucket/.cvsignore,
  simgear/bucket/Makefile.am, simgear/bucket/newbucket.cxx,
  simgear/bucket/newbucket.hxx, simgear/debug/.cvsignore,
  simgear/debug/Makefile.am, simgear/debug/debug_types.h,
  simgear/debug/logstream.cxx, simgear/debug/logstream.hxx,
  simgear/debug/logtest.cxx, simgear/magvar/.cvsignore,
  simgear/magvar/Makefile.am, simgear/magvar/coremag.cxx,
  simgear/magvar/coremag.hxx, simgear/magvar/magvar.cxx,
  simgear/magvar/magvar.hxx, simgear/magvar/testmagvar.cxx,
  simgear/math/.cvsignore, simgear/math/Makefile.am,
  simgear/math/interpolater.cxx, simgear/math/interpolater.hxx,
  simgear/math/leastsqs.cxx, simgear/math/leastsqs.hxx,
  simgear/math/linintp2.h, simgear/math/linintp2.inl,
  simgear/math/localconsts.hxx, simgear/math/point3d.hxx,
  simgear/math/polar3d.cxx, simgear/math/polar3d.hxx,
  simgear/math/sg_geodesy.cxx, simgear/math/sg_geodesy.hxx,
  simgear/math/sg_memory.h, simgear/math/sg_random.c,
  simgear/math/sg_random.h, simgear/math/sg_types.hxx,
  simgear/math/sphrintp.h, simgear/math/sphrintp.inl,
  simgear/math/vector.cxx, simgear/math/vector.hxx,
  simgear/misc/.cvsignore, simgear/misc/Makefile.am,
  simgear/misc/commands.cxx, simgear/misc/commands.hxx,
  simgear/misc/exception.cxx, simgear/misc/exception.hxx,
  simgear/misc/props.cxx, simgear/misc/props.hxx,
  simgear/misc/props_io.cxx, simgear/misc/props_io.hxx,
  simgear/misc/props_test.cxx, simgear/misc/sg_path.cxx,
  simgear/misc/sg_path.hxx, simgear/misc/sgstream.cxx,
  simgear/misc/sgstream.hxx, simgear/misc/stopwatch.hxx,
  simgear/misc/strutils.cxx, simgear/misc/strutils.hxx,
  simgear/misc/texcoord.cxx, simgear/misc/texcoord.hxx,
  simgear/misc/zfstream.cxx, simgear/misc/zfstream.hxx,
  simgear/screen/.cvsignore, simgear/screen/GLBitmaps.cxx,
  simgear/screen/GLBitmaps.h, simgear/screen/Makefile.am,
  simgear/screen/jpgfactory.cxx, simgear/screen/jpgfactory.hxx,
  simgear/screen/screen-dump.cxx, simgear/screen/screen-dump.hxx,
  simgear/screen/tr.cxx, simgear/screen/tr.h,
  simgear/screen/win32-printer.h, simgear/serial/.cvsignore,
  simgear/serial/Makefile.am, simgear/serial/serial.cxx,
  simgear/serial/serial.hxx, simgear/serial/testserial.cxx,
  simgear/timing/.cvsignore, simgear/timing/Makefile.am,
  simgear/timing/geocoord.cxx, simgear/timing/geocoord.h,
  simgear/timing/lowleveltime.cxx, simgear/timing/lowleveltime.h,
  simgear/timing/sg_time.cxx, simgear/timing/sg_time.hxx,
  simgear/timing/timestamp.cxx, simgear/timing/timestamp.hxx,
  simgear/timing/timezone.cxx, simgear/timing/timezone.h,
  simgear/ephemeris/.cvsignore, simgear/ephemeris/Makefile.am,
  simgear/ephemeris/celestialBody.cxx,
  simgear/ephemeris/celestialBody.hxx,
  simgear/ephemeris/ephemeris.cxx, simgear/ephemeris/ephemeris.hxx,
  simgear/ephemeris/jupiter.cxx, simgear/ephemeris/jupiter.hxx,
  simgear/ephemeris/mars.cxx, simgear/ephemeris/mars.hxx,
  simgear/ephemeris/mercury.cxx, simgear/ephemeris/mercury.hxx,
  simgear/ephemeris/moonpos.cxx, simgear/ephemeris/moonpos.hxx,
  simgear/ephemeris/neptune.cxx, simgear/ephemeris/neptune.hxx,
  simgear/ephemeris/pluto.hxx, simgear/ephemeris/saturn.cxx,
  simgear/ephemeris/saturn.hxx, simgear/ephemeris/star.cxx,
  simgear/ephemeris/star.hxx, simgear/ephemeris/stardata.cxx,
  simgear/ephemeris/stardata.hxx, simgear/ephemeris/uranus.cxx,
  simgear/ephemeris/uranus.hxx, simgear/ephemeris/venus.cxx,
  simgear/ephemeris/venus.hxx, simgear/io/.cvsignore,
  simgear/io/Makefile.am, simgear/io/decode_binobj.cxx,
  simgear/io/iochannel.cxx, simgear/io/iochannel.hxx,
  simgear/io/lowlevel.cxx, simgear/io/lowlevel.hxx,
  simgear/io/lowtest.cxx, simgear/io/sg_binobj.cxx,
  simgear/io/sg_binobj.hxx, simgear/io/sg_file.cxx,
  simgear/io/sg_file.hxx, simgear/io/sg_serial.cxx,
  simgear/io/sg_serial.hxx, simgear/io/sg_socket.cxx,
  simgear/io/sg_socket.hxx, simgear/io/sg_socket_udp.cxx,
  simgear/io/sg_socket_udp.hxx, simgear/io/socktest.cxx,
  simgear/io/tcp_client.cxx, simgear/io/tcp_server.cxx,
  simgear/route/.cvsignore, simgear/route/Makefile.am,
  simgear/route/route.cxx, simgear/route/route.hxx,
  simgear/route/routetest.cxx, simgear/route/waypoint.cxx,
  simgear/route/waypoint.hxx, simgear/route/waytest.cxx,
  simgear/scene/sky/.cvsignore, simgear/scene/sky/Makefile.am,
  simgear/scene/sky/cloud.cxx, simgear/scene/sky/cloud.hxx,
  simgear/scene/sky/dome.cxx, simgear/scene/sky/dome.hxx,
  simgear/scene/sky/moon.cxx, simgear/scene/sky/moon.hxx,
  simgear/scene/sky/oursun.cxx, simgear/scene/sky/oursun.hxx,
  simgear/scene/sky/sky.cxx, simgear/scene/sky/sky.hxx,
  simgear/scene/sky/sphere.cxx, simgear/scene/sky/sphere.hxx,
  simgear/scene/sky/stars.cxx, simgear/scene/sky/stars.hxx,
  simgear/threads/.cvsignore, simgear/threads/Makefile.am,
  simgear/threads/SGGuard.hxx, simgear/threads/SGQueue.hxx,
  simgear/threads/SGThread.cxx, simgear/threads/SGThread.hxx,
  simgear/xml/.cvsignore, simgear/xml/Makefile.am,
  simgear/xml/asciitab.h, simgear/xml/easyxml.cxx,
  simgear/xml/easyxml.hxx, simgear/xml/hashtable.c,
  simgear/xml/hashtable.h, simgear/xml/iasciitab.h,
  simgear/xml/latin1tab.h, simgear/xml/nametab.h,
  simgear/xml/sample.xml, simgear/xml/testEasyXML.cxx,
  simgear/xml/utf8tab.h, simgear/xml/xmldef.h,
  simgear/xml/xmlparse.c, simgear/xml/xmlparse.h,
  simgear/xml/xmlrole.c, simgear/xml/xmlrole.h,
  simgear/xml/xmltok.c, simgear/xml/xmltok.h,
  simgear/xml/xmltok_impl.c, simgear/xml/xmltok_impl.h,
  simgear/xml/xmltok_ns.c: Initial revsion of Simgear-0.3.0
