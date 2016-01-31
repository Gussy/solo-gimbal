#!/usr/bin/env python

import httplib, json, re, sys, argparse
import urllib as urllib
urllib.parse = urllib

# Some parts below borrowed from https://github.com/jpaugh/agithub
class Content(object):
    '''
    Decode a response from the server, respecting the Content-Type field
    '''
    def __init__(self, response):
        self.response = response
        self.body = response.read()
        (self.mediatype, self.encoding) = self.get_ctype()

    def get_ctype(self):
        '''Split the content-type field into mediatype and charset'''
        ctype = self.response.getheader('Content-Type')

        start = 0
        end = 0
        try:
            end = ctype.index(';')
            mediatype = ctype[:end]
        except:
            mediatype = 'x-application/unknown'

        try:
            start = 8 + ctype.index('charset=', end)
            end = ctype.index(';', start)
            charset = ctype[start:end].rstrip()
        except:
            charset = 'ISO-8859-1' #TODO

        return (mediatype, charset)

    def decode_body(self):
        '''
        Decode (and replace) self.body via the charset encoding
        specified in the content-type header
        '''
        self.body = self.body.decode(self.encoding)


    def processBody(self):
        '''
        Retrieve the body of the response, encoding it into a usuable
        form based on the media-type (mime-type)
        '''
        handlerName = self.mangled_mtype()
        handler = getattr(self, handlerName, self.x_application_unknown)
        return handler()


    def mangled_mtype(self):
        '''
        Mangle the media type into a suitable function name
        '''
        return self.mediatype.replace('-','_').replace('/','_')


    ## media-type handlers

    def x_application_unknown(self):
        '''Handler for unknown media-types'''
        return self.body

    def application_json(self):
        '''Handler for application/json media-type'''
        self.decode_body()

        try:
            pybody = json.loads(self.body)
        except ValueError:
            pybody = self.body

        return pybody

class Client(object):
    def __init__(self, token=''):
        if token is None:
            raise TypeError("You need to provide a token")
        self.default_headers = dict()
        self.default_headers['user-agent'] = 'python2.7'
        self.auth_header = 'Token %s' % token
        self.api_url = 'api.github.com'

    def get(self, url, headers={}, **params):
        url += self.urlencode(params)
        return self.request('GET', url, None, headers)

    def request(self, method, url, body, headers):
        headers = self._fix_headers(headers)

        if self.auth_header:
            headers['authorization'] = self.auth_header

        conn = httplib.HTTPSConnection(self.api_url)
        conn.request(method, url, body, headers)
        response = conn.getresponse()
        status = response.status
        content = Content(response)
        self.headers = response.getheaders()

        conn.close()
        return status, content.processBody()

    def _fix_headers(self, headers):
        # Convert header names to a uniform case
        tmp_dict = {}
        for k,v in headers.items():
            tmp_dict[k.lower()] = v
        headers = tmp_dict

        # Add default headers (if unspecified)
        for k,v in self.default_headers.items():
            if k not in headers:
                headers[k] = v
        return headers

    def urlencode(self, params):
        if not params:
            return ''
        return '?' + urllib.parse.urlencode(params)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo_owner", help="git repository owner", required=True)
    parser.add_argument("--repo_name", help="git repository name", required=True)
    args = parser.parse_args()

    github = Client(token='1a9dcbc55435ca9a691d0f578ba727cbde15052b')

    authors = list()
    for sha in sys.stdin:
        status, data = github.get('/repos/%s/%s/commits/%s' % (args.repo_owner, args.repo_name, sha.strip()))
        user = data['author']['login']
        if ("@%s" % user) not in authors:
            authors.append("@%s" % user)

    print(', '.join(authors))
