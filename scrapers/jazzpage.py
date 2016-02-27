#!/usr/bin/env python
"""
Scraper for The Jazz Page MIDI recordings
"""
import urllib2
import urllib
from BeautifulSoup import BeautifulSoup

req = urllib2.Request("http://www.thejazzpage.de/midiinfo.html")
response = urllib2.urlopen(req)
page_contents = response.read()

base_curl_url = "http://www.thejazzpage.de/"

soup = BeautifulSoup(page_contents)
for link in soup.findAll('a', href=True):
	if link['href'].endswith('.mid'):
		print "Scraping " + link['href']
		full_url = base_curl_url + urllib.quote(link['href'])
		print full_url
		read_file = urllib.URLopener()
		try:
			read_file.retrieve(full_url, "jazzpage/" + link['href'].split('/')[1])
		except:
			continue


